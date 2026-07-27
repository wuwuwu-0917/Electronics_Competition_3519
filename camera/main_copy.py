
# ============================================================
# 钢球检测主程序
#   - YOLOv5 推理 + 类别白名单 + 手动NMS
#   - 贪心匹配多目标追踪器 + 确认/滑行机制
#   - 数量中值滤波 + FPS统计
# ============================================================

from maix import camera, display, image, nn, app, uart, pinmap, err, time
import struct, os

# ---- 检测参数 -------------------------------------------------
CONF_TH = 0.50          # 置信度阈值
IOU_TH  = 0.70          # 内置 NMS IoU 阈值
MANUAL_NMS_TH = 0.50    # 手动 NMS IoU 阈值
TARGET_LABELS = ["sb"]  # 目标类别白名单

# ---- 追踪器参数 -----------------------------------------------
CONFIRM_HITS = 2        # 连续命中帧数 → 确认目标
COAST_MAX    = 5        # 确认目标允许丢失的最大帧数
MATCH_DIST   = 80       # 匹配门限 (像素)，帧间正常移动不会超过此距离
MERGE_DIST   = 15       # 合并门限 (像素)，两个确认目标小于此距离才合并

# ---- UART 配置 ------------------------------------------------
# MaixCAM2: U0T/U0R 即 UART0
UART_DEV  = "/dev/ttyS0"
UART_BAUD = 115200
ENABLE_UART = True       # 串口发送开关

# ---- CRC16 表 -------------------------------------------------
_CRC16_TAB = []
for _i in range(256):
    _crc = _i << 8
    for _b in range(8):
        _crc = (_crc << 1) ^ 0x1021 if _crc & 0x8000 else _crc << 1
    _CRC16_TAB.append(_crc & 0xFFFF)

def crc16(data):
    """CRC16-CCITT (XMODEM)"""
    crc = 0
    for byte in data:
        crc = ((crc << 8) ^ _CRC16_TAB[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


# ================================================================
# 工具函数
# ================================================================

def build_packet(tracks, img_w, img_h, class_id=0):
    """
    组包：帧头 + 数量 + 目标数据 + CRC16 + 帧尾
    每目标: 2B cx(LE) + 2B cy(LE) + 2B w(LE) + 2B h(LE) + 1B class + 1B score%
    帧格式: AA 55 [count] [data...] [crc16 LE] DD
    """
    payload = struct.pack("<B", len(tracks))  # 目标数量
    for t in tracks:
        score_pct = min(99, int(t.score * 100))  # 置信度百分比 0-99
        payload += struct.pack("<HHHHBB",
                               int(t.cx), int(t.cy),
                               int(t.w), int(t.h),
                               class_id, score_pct)
    crc = crc16(payload)
    return b'\xAA\x55' + payload + struct.pack("<H", crc) + b'\xDD'


def filter_by_class(objs, labels, target_labels):
    if not target_labels:
        return objs, 0
    filtered = []
    rejected = 0
    for obj in objs:
        class_name = labels[obj.class_id] if obj.class_id < len(labels) else "???"
        if class_name in target_labels:
            filtered.append(obj)
        else:
            rejected += 1
    return filtered, rejected


def calc_iou(obj_a, obj_b):
    x1 = max(obj_a.x, obj_b.x)
    y1 = max(obj_a.y, obj_b.y)
    x2 = min(obj_a.x + obj_a.w, obj_b.x + obj_b.w)
    y2 = min(obj_a.y + obj_a.h, obj_b.y + obj_b.h)
    inter_w = max(0, x2 - x1)
    inter_h = max(0, y2 - y1)
    inter_area = inter_w * inter_h
    area_a = obj_a.w * obj_a.h
    area_b = obj_b.w * obj_b.h
    union_area = area_a + area_b - inter_area
    return inter_area / union_area if union_area > 0 else 0.0


def manual_nms(objs, iou_th):
    if len(objs) <= 1:
        return objs, 0
    sorted_objs = sorted(objs, key=lambda o: o.score, reverse=True)
    keep = []
    suppressed = 0
    for obj in sorted_objs:
        overlap = any(calc_iou(obj, k) > iou_th for k in keep)
        if overlap:
            suppressed += 1
        else:
            keep.append(obj)
    return keep, suppressed


# ================================================================
# 追踪器 (贪心匹配, O(n²))
# ================================================================

class Track:
    """单个目标的追踪状态"""
    __slots__ = ("cx", "cy", "w", "h", "score",
                 "vx", "vy", "hits", "misses", "confirmed")

    def __init__(self, cx, cy, w, h, score):
        self.cx, self.cy = cx, cy
        self.w, self.h = w, h
        self.score = score
        self.vx, self.vy = 0.0, 0.0
        self.hits = 1
        self.misses = 0
        self.confirmed = False

    def predict(self):
        """预测下一帧的中心位置"""
        return self.cx + self.vx, self.cy + self.vy

    def to_rect(self):
        """返回 (x, y, w, h) 整数坐标"""
        return (int(self.cx - self.w / 2),
                int(self.cy - self.h / 2),
                int(self.w), int(self.h))


class Tracker:
    """多目标追踪器：贪心匹配 + 确认/滑行"""

    def __init__(self):
        self.tracks = []

    def update(self, detections):
        """
        detections: [(cx, cy, w, h, score), ...]
        返回: 已确认的 Track 列表
        """
        n_tracks = len(self.tracks)
        n_dets = len(detections)

        # ---- 贪心匹配 (O(n²)，速度快，钢球场景足够用) ----
        d2max = MATCH_DIST * MATCH_DIST
        # 为每个track找最近的detection
        track_to_det = [-1] * n_tracks
        det_to_track = [-1] * n_dets
        # track按置信度降序（优先匹配高分追踪器）
        track_order = sorted(range(n_tracks), key=lambda i: self.tracks[i].score, reverse=True)
        for ti in track_order:
            t = self.tracks[ti]
            px, py = t.predict()
            best_di = -1
            best_dd = d2max + 1
            for di, d in enumerate(detections):
                if det_to_track[di] >= 0:  # 已被占用
                    continue
                dd = (d[0] - px) ** 2 + (d[1] - py) ** 2
                if dd < best_dd:
                    best_dd = dd
                    best_di = di
            if best_di >= 0:
                track_to_det[ti] = best_di
                det_to_track[best_di] = ti

        matched_track = [False] * n_tracks
        matched_det = [False] * n_dets

        # ---- 更新匹配成功的追踪器 ----
        for ti in range(n_tracks):
            di = track_to_det[ti]
            if di < 0:  # 未匹配到任何检测
                continue

            d = detections[di]
            matched_track[ti] = True
            matched_det[di] = True
            t = self.tracks[ti]

            # 位置直接用检测值（零延迟），尺寸做轻量平滑
            t.cx, t.cy = d[0], d[1]
            t.w  = 0.6 * d[2] + 0.4 * t.w
            t.h  = 0.6 * d[3] + 0.4 * t.h
            t.score = d[4]
            t.hits += 1
            t.misses = 0

            if t.hits >= CONFIRM_HITS:
                t.confirmed = True

        # ---- 未匹配的追踪器：原地保持，不等速滑行（避免急转弯甩飞） ----
        for ti in range(n_tracks):
            if matched_track[ti]:
                continue
            t = self.tracks[ti]
            t.misses += 1

        # ---- 未匹配的检测：孵化新追踪器 ----
        merge_d2 = MERGE_DIST * MERGE_DIST
        for di in range(n_dets):
            if matched_det[di]:
                continue
            d = detections[di]

            # 检查是否靠近某个滑行中的确认追踪器 → 接管
            absorbed = False
            for t in self.tracks:
                if t.misses > 0 and t.confirmed:
                    dd = (d[0] - t.cx) ** 2 + (d[1] - t.cy) ** 2
                    if dd < merge_d2:
                        t.cx, t.cy = d[0], d[1]
                        t.w, t.h = d[2], d[3]
                        t.score = d[4]
                        t.hits += 1
                        t.misses = 0
                        absorbed = True
                        break
            if not absorbed:
                self.tracks.append(Track(*d))

        # ---- 清理死亡追踪器 ----
        survivors = []
        for t in self.tracks:
            if t.misses <= COAST_MAX:
                survivors.append(t)
        self.tracks = survivors

        # ---- 合并重叠的确认追踪器 ----
        confirmed = [t for t in self.tracks if t.confirmed]
        for i in range(len(confirmed) - 1, -1, -1):
            for j in range(i):
                a, b = confirmed[i], confirmed[j]
                dd = (a.cx - b.cx) ** 2 + (a.cy - b.cy) ** 2
                if dd < merge_d2:
                    loser = a if a.hits < b.hits else b
                    loser.misses = COAST_MAX + 1
        self.tracks = [t for t in self.tracks if t.misses <= COAST_MAX]

        return [t for t in self.tracks if t.confirmed]


# ================================================================
# 模型加载
# ================================================================
model_path = "model_303178.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/mymodel/steelball/model_me/model_303178.mud"

detector = nn.YOLOv5(model=model_path)
print(f"[detector] model: {model_path}")
print(f"[detector] labels: {detector.labels}")
print(f"[detector] input: {detector.input_width()}x{detector.input_height()}")
print(f"[detector] conf_th={CONF_TH}, iou_th={IOU_TH}")
print(f"[tracker] confirm={CONFIRM_HITS}, coast={COAST_MAX}, "
      f"match_dist={MATCH_DIST}, merge_dist={MERGE_DIST}")

cam = camera.Camera(detector.input_width(), detector.input_height(),
                    detector.input_format())
W, H = detector.input_width(), detector.input_height()
dis = display.Display()

# UART 初始化（UART0 系统默认，无需 pinmap）
ser = None
if ENABLE_UART:
    ser = uart.UART(UART_DEV, UART_BAUD)
    print(f"[uart] {UART_DEV} @ {UART_BAUD} baud")

tracker = Tracker()

# 数量滤波（取最近10帧的中值，消除2↔3闪烁）
COUNT_WINDOW = 10
count_history = []

def count_filter(new_count):
    """中值滤波：返回最近N帧的中位数"""
    count_history.append(new_count)
    if len(count_history) > COUNT_WINDOW:
        count_history.pop(0)
    sorted_vals = sorted(count_history)
    mid = len(sorted_vals) // 2
    return sorted_vals[mid]

# FPS 统计
last_fps_ticks = time.ticks_ms()
frame_count = 0
fps_val = 0.0

# ================================================================
# 主循环
# ================================================================
while not app.need_exit():
    img = cam.read()
    frame_count += 1

    # 1. YOLO 推理
    raw_objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)

    # 2. 类别过滤
    objs, class_rejected = filter_by_class(raw_objs, detector.labels, TARGET_LABELS)

    # 3. 手动 NMS
    objs, nms_suppressed = manual_nms(objs, MANUAL_NMS_TH)

    # 4. 转换为追踪器输入格式: (cx, cy, w, h, score)
    detections = [(o.x + o.w // 2, o.y + o.h // 2, o.w, o.h, o.score) for o in objs]

    # 5. 追踪器更新 → 只返回已确认的目标
    confirmed_tracks = tracker.update(detections)

    # 6. 数量滤波（中值滤波消除闪烁）
    raw_count = len(confirmed_tracks)
    stable_count = count_filter(raw_count)

    # 上报：CRC16组包通过UART发送
    if stable_count > 0 and raw_count > 0:
        sorted_tracks = sorted(confirmed_tracks, key=lambda t: t.score, reverse=True)
        report_tracks = sorted_tracks[:stable_count]
        packet = build_packet(report_tracks, W, H)
        if ser is not None:
            ser.write(packet)
        # 每秒打印一次十六进制包内容（调试用，确认后注释掉）
        if frame_count % 30 == 0:
            hex_str = " ".join([f"{b:02X}" for b in packet])
            print(f"[UART] {len(packet)}B: {hex_str}")

    # 7. 绘制（只画已确认追踪目标，一球一框）
    for t in confirmed_tracks:
        x, y, w, h = t.to_rect()
        img.draw_rect(x, y, w, h, image.COLOR_GREEN, 2)
        img.draw_string(x, max(0, y - 14), f"{t.score:.2f}", image.COLOR_GREEN)

    # 8. 状态叠加
    all_tracks = len(tracker.tracks)
    uart_mark = "TX" if (ser is not None and stable_count > 0) else "---"
    status = (f"det:{len(objs)} trk:{all_tracks} cfm:{raw_count}->{stable_count} "
              f"uart:{uart_mark} fps:{fps_val:.0f}")
    img.draw_string(4, 4, status, image.COLOR_GREEN)

    dis.show(img)

    # 9. FPS + 每秒打印坐标
    if frame_count % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_fps_ticks)
        fps_val = 30000.0 / elapsed
        last_fps_ticks = now
        # 打印每个目标的中心坐标
        coords = ", ".join([f"#{i}({int(t.cx)},{int(t.cy)})s{t.score:.2f}"
                           for i, t in enumerate(confirmed_tracks)])
        print(f"[{fps_val:.0f}fps] {len(confirmed_tracks)} targets: {coords}")
