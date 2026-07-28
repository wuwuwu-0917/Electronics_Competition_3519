# ============================================================
# 钢球-中心距离标注程序
#   - 绿框标出钢球，红线连接画面中心
#   - 三条竖直虚线（1/4, 1/2, 3/4 分界线）
#   - 每球标注距中心距离 (D / dx / dy)
#   - SHOW_ONLY_NEAREST: 1=只标最近球 0=标全部
#   - ball_zone: -2/-1/0/1/2 巡线引导变量
# ============================================================

from maix import camera, display, image, nn, app, time, uart
import math, os, struct

# ---- 检测参数 -------------------------------------------------
CONF_TH = 0.60          # 置信度阈值（钢球训练充分可设0.60~0.70）
IOU_TH  = 0.50          # 内置 NMS IoU 阈值（替代手动NMS）
TARGET_LABELS = ["sb"]  # 目标类别白名单

# ---- 追踪器参数 -----------------------------------------------
CONFIRM_HITS = 1        # 连续命中帧数 → 确认目标（1=首帧确认，最快响应）
COAST_MAX    = 3        # 确认目标允许丢失的最大帧数（缩小，死追踪器快速清除）
MATCH_DIST   = 80       # 匹配门限 (像素)，帧间正常移动不会超过此距离
MERGE_DIST   = 15       # 合并门限 (像素)，两个确认目标小于此距离才合并

# ---- 颜色常量 -------------------------------------------------
COLOR_GREEN   = image.COLOR_GREEN
COLOR_RED     = image.COLOR_RED
COLOR_YELLOW  = image.COLOR_YELLOW
COLOR_WHITE   = image.COLOR_WHITE
COLOR_CROSS   = image.Color(0, 255, 255)    # 青色（十字准星）

# ---- 绘制开关 -------------------------------------------------
SHOW_ONLY_NEAREST = 1    # 1=只标最近球  0=标全部球

# ---- 巡线引导 ------------------------------------------------
CENTER_THRESH = 7        # 球框边缘距中心线 ±7px 内 → zone=0

# ---- UART 配置 ------------------------------------------------
UART_DEV  = "/dev/ttyS0"
UART_BAUD = 115200
ENABLE_UART = True

# ---- CRC16 表 -------------------------------------------------
_CRC16_TAB = []
for _i in range(256):
    _crc = _i << 8
    for _b in range(8):
        _crc = (_crc << 1) ^ 0x1021 if _crc & 0x8000 else _crc << 1
    _CRC16_TAB.append(_crc & 0xFFFF)

def crc16(data):
    crc = 0
    for byte in data:
        crc = ((crc << 8) ^ _CRC16_TAB[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc

def build_ball_packet(has_ball, ball_zone, cx, cy):
    """
    组包: AA 55 | has_ball(1B) | zone(1B signed) | cx(2B LE) | cy(2B LE) | CRC16(2B LE) | DD
    共 11 字节
    """
    payload = struct.pack("<BbHH",
                          has_ball & 1,
                          max(-128, min(127, ball_zone)),
                          int(cx), int(cy))
    crc = crc16(payload)
    return b'\xAA\x55' + payload + struct.pack("<H", crc) + b'\xDD'


# ================================================================
# 追踪器（与 ball_detect.py 同）
# ================================================================

class Track:
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
        return self.cx + self.vx, self.cy + self.vy

    def to_rect(self):
        return (int(self.cx - self.w / 2),
                int(self.cy - self.h / 2),
                int(self.w), int(self.h))


class Tracker:
    def __init__(self):
        self.tracks = []

    def update(self, detections):
        n_tracks = len(self.tracks)
        n_dets = len(detections)

        d2max = MATCH_DIST * MATCH_DIST
        track_to_det = [-1] * n_tracks
        det_to_track = [-1] * n_dets
        track_order = sorted(range(n_tracks), key=lambda i: self.tracks[i].score, reverse=True)

        for ti in track_order:
            t = self.tracks[ti]
            px, py = t.predict()                # 匀速预测位置
            best_di = -1
            best_dd = d2max + 1
            for di, d in enumerate(detections):
                if det_to_track[di] >= 0:
                    continue
                # 取预测位置和原地位置中更近的那个 → 变向也能匹配上
                dd = min((d[0] - px) ** 2 + (d[1] - py) ** 2,
                         (d[0] - t.cx) ** 2 + (d[1] - t.cy) ** 2)
                if dd < best_dd:
                    best_dd = dd
                    best_di = di
            if best_di >= 0:
                track_to_det[ti] = best_di
                det_to_track[best_di] = ti

        matched_track = [False] * n_tracks
        matched_det = [False] * n_dets

        for ti in range(n_tracks):
            di = track_to_det[ti]
            if di < 0:
                continue
            d = detections[di]
            matched_track[ti] = True
            matched_det[di] = True
            t = self.tracks[ti]

            # ★ 先算速度，再更新位置（顺序不能反）
            t.vx = 0.7 * (d[0] - t.cx) + 0.3 * t.vx  # EMA平滑速度
            t.vy = 0.7 * (d[1] - t.cy) + 0.3 * t.vy
            t.cx, t.cy = d[0], d[1]
            t.w  = 0.6 * d[2] + 0.4 * t.w
            t.h  = 0.6 * d[3] + 0.4 * t.h
            t.score = d[4]
            t.hits += 1
            t.misses = 0
            if t.hits >= CONFIRM_HITS:
                t.confirmed = True

        for ti in range(n_tracks):
            if matched_track[ti]:
                continue
            self.tracks[ti].misses += 1

        merge_d2 = MERGE_DIST * MERGE_DIST
        for di in range(n_dets):
            if matched_det[di]:
                continue
            d = detections[di]
            absorbed = False
            for t in self.tracks:
                if t.misses > 0 and t.confirmed:
                    dd = (d[0] - t.cx) ** 2 + (d[1] - t.cy) ** 2
                    if dd < merge_d2:
                        t.vx = 0.7 * (d[0] - t.cx) + 0.3 * t.vx
                        t.vy = 0.7 * (d[1] - t.cy) + 0.3 * t.vy
                        t.cx, t.cy = d[0], d[1]
                        t.w, t.h = d[2], d[3]
                        t.score = d[4]
                        t.hits += 1
                        t.misses = 0
                        absorbed = True
                        break
            if not absorbed:
                self.tracks.append(Track(*d))

        survivors = []
        for t in self.tracks:
            if t.misses <= COAST_MAX:
                survivors.append(t)
        self.tracks = survivors

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
# 绘制工具
# ================================================================

def draw_dashed_line(img, x0, y0, x1, y1, color, dash_len=6, gap_len=4):
    """画虚线：水平或垂直方向"""
    if y0 == y1:
        # 水平虚线
        x = x0
        end = x1
        while x < end:
            xe = min(x + dash_len, end)
            img.draw_line(x, y0, xe, y0, color, 1)
            x = xe + gap_len
    elif x0 == x1:
        # 垂直虚线
        y = y0
        end = y1
        while y < end:
            ye = min(y + dash_len, end)
            img.draw_line(x0, y, x0, ye, color, 1)
            y = ye + gap_len


# ================================================================
# 巡线引导：仅对标出框的球进行位置判断
# ================================================================

def calc_ball_zone(t, cx, w):
    """
    单球判断逻辑:
      球心距中心 ≤ CENTER_THRESH       → 0
      球框完全在 Q1 左侧               → -2
      球框与 Q1~Q2 有交集（含跨越Q1）  → -1
      球框完全在 Q3 右侧               →  2
      球框与 Q2~Q3 有交集（含跨越Q3）  →  1
    """
    if t is None:
        return 0

    Q1, Q2, Q3 = w // 4, w // 2, w * 3 // 4
    left  = t.cx - t.w / 2
    right = t.cx + t.w / 2

    if abs(t.cx - cx) <= CENTER_THRESH:
        return 0
    if right <= Q2:
        return -2 if right <= Q1 else -1
    if left >= Q2:
        return 2 if left >= Q3 else 1
    return 0


# ================================================================
# 模型加载
# ================================================================
model_path = "model_303178.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/mymodel/steelball/model_me/model_303178.mud"

detector = nn.YOLOv5(model=model_path)

cam = camera.Camera(detector.input_width(), detector.input_height(),
                    detector.input_format())
W, H = detector.input_width(), detector.input_height()
dis = display.Display()

# UART 初始化
ser = None
if ENABLE_UART:
    ser = uart.UART(UART_DEV, UART_BAUD)

# 中心点
CX = W // 2
CY = H // 2

# 三条四分线 X 坐标（竖线，分布在水平方向）
Q1_X = W // 4           # 左 1/4 线
Q2_X = W // 2           # 中线 (1/2)
Q3_X = W * 3 // 4       # 右 3/4 线

tracker = Tracker()

# FPS
last_fps_ticks = time.ticks_ms()
frame_count = 0
fps_val = 0.0


# ================================================================
# 主循环
# ================================================================
while not app.need_exit():
    img = cam.read()
    frame_count += 1

    # ---- YOLO 推理（必须在画任何叠加线之前，否则干扰检测） ----
    raw_objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)

    # ---- 背景叠加（YOLO 之后画，不干扰检测） ----
    draw_dashed_line(img, Q1_X, 0, Q1_X, H, COLOR_YELLOW, dash_len=8, gap_len=6)
    draw_dashed_line(img, Q2_X, 0, Q2_X, H, COLOR_YELLOW, dash_len=8, gap_len=6)
    draw_dashed_line(img, Q3_X, 0, Q3_X, H, COLOR_YELLOW, dash_len=8, gap_len=6)
    img.draw_line(CX - 10, CY, CX + 10, CY, COLOR_CROSS, 1)
    img.draw_line(CX, CY - 10, CX, CY + 10, COLOR_CROSS, 1)

    # ---- 类别过滤 ----
    objs = [o for o in raw_objs
            if o.class_id < len(detector.labels)
            and detector.labels[o.class_id] in TARGET_LABELS]

    # ---- 转追踪格式 ----
    detections = [(o.x + o.w // 2, o.y + o.h // 2, o.w, o.h, o.score) for o in objs]

    # ---- 追踪器 ----
    confirmed_tracks = tracker.update(detections)

    # ---- 最近球（实时选几何最近） ----
    nearest = None
    if confirmed_tracks:
        nearest = min(confirmed_tracks,
                      key=lambda t: math.hypot(t.cx - CX, t.cy - CY))

    # ---- 确定要绘制的目标 ----
    draw_tracks = [nearest] if (SHOW_ONLY_NEAREST and nearest) else confirmed_tracks

    # ---- 巡线引导（始终用最近球） ----
    ball_zone = calc_ball_zone(nearest, CX, W)
    has_ball = 1 if nearest else 0

    # ---- UART 发送（has_ball, zone, cx, cy） ----
    tx_count = 0
    if ser is not None:
        tx_cx = int(nearest.cx) if nearest else 0
        tx_cy = int(nearest.cy) if nearest else 0
        packet = build_ball_packet(has_ball, ball_zone, tx_cx, tx_cy)
        ser.write(packet)
        tx_count = len(packet)

    # ---- UART 接收 MCU 应答 ----
    rx_count = 0
    if ser is not None:
        rx_bytes = ser.read()
        if rx_bytes:
            rx_count = len(rx_bytes)

    # ---- 状态打印 ----
    if frame_count % 30 == 0:
        print(f"[STATUS] frm={frame_count} tx={tx_count}B rx={rx_count}B "
              f"Z={ball_zone:+d} D={has_ball}")

    # ---- 绘制目标 ----
    for t in draw_tracks:
        bx, by, bw, bh = t.to_rect()

        # ① 绿框
        img.draw_rect(bx, by, bw, bh, COLOR_GREEN, 2)

        # ② 红线：球心 → 画面中心
        img.draw_line(int(t.cx), int(t.cy), CX, CY, COLOR_RED, 1)

        # ③ 距离标注：dx, dy 像素
        dx = int(t.cx - CX)
        dy = int(t.cy - CY)
        dist = int(math.hypot(dx, dy))

        label_x = bx + bw + 2
        label_y = max(0, by - 28)
        img.draw_string(label_x, label_y,
                        f"D:{dist}", COLOR_YELLOW)
        img.draw_string(label_x, label_y + 12,
                        f"dx:{dx:+d}", COLOR_YELLOW)
        img.draw_string(label_x, label_y + 24,
                        f"dy:{dy:+d}", COLOR_YELLOW)

    # ---- 状态叠加（左上角竖排） ----
    img.draw_string(4, 4,  f"Z:{ball_zone:+d}", COLOR_WHITE)
    img.draw_string(4, 20, f"F:{fps_val:.0f}",  COLOR_WHITE)
    img.draw_string(4, 36, f"D:{has_ball}",     COLOR_WHITE)
    img.draw_string(4, 52, f"B:{len(confirmed_tracks)}", COLOR_WHITE)

    dis.show(img)

    # ---- FPS ----
    if frame_count % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_fps_ticks)
        fps_val = 30000.0 / elapsed
        last_fps_ticks = now
