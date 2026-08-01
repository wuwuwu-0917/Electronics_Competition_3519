
# ============================================================
# YOLO 钢球检测 + 绿色管道透视矫正 (MaixPy)
#   - 上电识别绿色管道 → 透视变换铺平 (仅一次)
#   - LED 补光 + UART 发送 (CRC16 校验包)
#   - YOLO 在原图上推理, warp 图用于参考线和位置计算
# ============================================================

from maix import camera, display, image, nn, app, comm, time, uart, gpio, pinmap, pwm, err, sys, touchscreen
import struct, os, json, threading, gc
from http.server import HTTPServer, BaseHTTPRequestHandler

ThreadingHTTPServer = None
try:
    from http.server import ThreadingHTTPServer as _THS
    ThreadingHTTPServer = _THS
except ImportError:
    try:
        from socketserver import ThreadingMixIn
        class _ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
            daemon_threads = True
        ThreadingHTTPServer = _ThreadingHTTPServer
    except ImportError:
        class _ManualThreadingHTTPServer(HTTPServer):
            daemon_threads = True
            def process_request(self, request, client_address):
                t = threading.Thread(target=self._handle_request,
                                    args=(request, client_address),
                                    daemon=True)
                t.start()
            def _handle_request(self, request, client_address):
                try:
                    self.finish_request(request, client_address)
                finally:
                    self.shutdown_request(request)
        ThreadingHTTPServer = _ManualThreadingHTTPServer

report_on = True
APP_CMD_DETECT_RES = 0x02

def encode_objs(objs):
    '''
        encode objs info to bytes body for protocol
        2B x(LE) + 2B y(LE) + 2B w(LE) + 2B h(LE) + 2B idx + 4B score(float) ...
    '''
    body = b''
    for obj in objs:
        body += struct.pack("<hhHHHf", obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score)
    return body

model_path = "model_310357.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/mymodel/steelball/model_me2/model_310357.mud"
detector = nn.YOLOv5(model=model_path)

# ---- 绿色阈值 (LAB) --------------------------------------------
GREEN_THRESHOLD = (0, 100, -128, -36, -128, 127)
MIN_AREA = 150
MERGE    = False
MARGIN   = 10

# ---- Web 推流配置 ----------------------------------------------
WEB_PORT = 5000
JPEG_QUALITY = 60
WEB_FRAME_SKIP = 2   # 每N帧编码一次JPEG (降CPU, 追踪不受影响)
WEB_ENABLE = True
RECORD_DIR = "/root/videos"
RECORD_ENABLE = False   # 设备端不录制, 浏览器端录制代替

# ---- 标尺参数 (管子长度 cm) -------------------------------------
PIPE_LENGTH = 25.0   # cm
PIPE_INSET_LEFT  = 8    # px, 管道左端内缩
PIPE_INSET_RIGHT = 12   # px, 管道右端内缩

# ---- LED 补光 --------------------------------------------------
LED_ON = True
if LED_ON:
    pin_name = "B25" if sys.device_id() == "maixcam2" else "B3"
    gpio_id  = "GPIOB25" if sys.device_id() == "maixcam2" else "GPIOB3"
    err.check_raise(pinmap.set_pin_function(pin_name, gpio_id), "set pin failed")
    led = gpio.GPIO(gpio_id, gpio.Mode.OUT)
    led.value(1)  # 常亮补光, 增强钢珠反光

# ---- 舵机 + 平衡控制器 (MaixCAM2 直驱, 不走通信) ----------------
SERVO_ENABLE  = True
SERVO_PIN     = "A31"
SERVO_PWM_ID  = 7
SERVO_FREQ    = 300       # 数字舵机 300Hz
SERVO_MIN_DUTY = 15.0     # 0° (0.5ms / 3.333ms周期)
SERVO_MAX_DUTY = 75.0     # 180° (2.5ms / 3.333ms周期)

# ---- 单套 PD (2026-08-01 简化) ----
BAL_CENTER    = 90.0
BAL_TRIM      = 0.0    # 仅Mode1使用, Mode2/3不需要
BAL_MAX_ANGLE = 140.0
BAL_MIN_ANGLE = 40.0
BAL_SLOPE_POS = 2.0  # 正目标(右边)前馈, 右边机械需要更弱前馈
BAL_SLOPE_NEG = 3.5  # 负目标(左边)前馈
KP = 6.0;   KD = 3.0;   DB = 0.1;   RATE = 300.0  # 300Hz舵机: 增益减半补偿6x响应速度
CURRENT_MODE = 1
# 内部状态
_pid_integral   = 0.0
_pid_last_out   = 90.0
_pid_last_ms    = 0
_bias_ramp      = 1.0   # bias缓启动: 0→1, 防超调

# ---- Mode 3 目标点 (触摸屏设定) ----
_target_cm      = 0.0    # 目标位置 (cm), 默认中心
_target_touched = False  # 用户是否已点击设置目标

# ---- 舵机初始化 --------------------------------------------------
if SERVO_ENABLE:
    err.check_raise(
        pinmap.set_pin_function(SERVO_PIN, f"PWM{SERVO_PWM_ID}"),
        f"[Servo] 设置 {SERVO_PIN} → PWM{SERVO_PWM_ID} 失败"
    )
    servo = pwm.PWM(SERVO_PWM_ID, freq=SERVO_FREQ, duty=0, enable=True)
    _mid_duty = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * BAL_CENTER / 180.0
    servo.duty(_mid_duty)
    print(f"[INFO] Servo on {SERVO_PIN} (PWM{SERVO_PWM_ID})  "
          f"freq={SERVO_FREQ}Hz  center={BAL_CENTER}deg  duty={_mid_duty:.2f}%")
    print(f"[INFO] PD: KP={KP} KD={KD} DB={DB} Rate={RATE}  SlopeP={BAL_SLOPE_POS} SlopeN={BAL_SLOPE_NEG}")
    print("[INFO] Servo ready.")

# ---- 触摸屏 ------------------------------------------------------
ts = touchscreen.TouchScreen()
_mode_btns = []   # 触摸按钮区域 [(x,y,w,h, label, mode_id), ...]
_fine_btns = []   # Mode 3 微调控件 [(x,y,w,h, action), ...]
_fine_held_act = ''  # 当前按住的动作(防抖: 按住只触发一次)

# ---- Mode 2 自动序列状态 ----
_m2_state       = 0    # 0=空闲, 1=等归零, 2=往+5, 3=往-5, 4=完成
_m2_timer_start = 0    # 序列开始时刻(ticks_ms)
_m2_timer_ms    = 0    # 当前/最终耗时(ms)
_m2_start_held  = False  # Start按键防抖

# ---- UART 配置 --------------------------------------------------
UART_DEV  = "/dev/ttyS0"
UART_BAUD = 115200
ENABLE_UART = True

# ---- CRC16 表 ----------------------------------------------------
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

def build_packet(pos_cm):
    """组包: AA 55 | pos_cm(2B signed LE) | CRC16(2B LE) | DD  (7字节)"""
    payload = struct.pack("<h", int(pos_cm * 100))  # 厘米×100
    crc = crc16(payload)
    return b'\xAA\x55' + payload + struct.pack("<H", crc) + b'\xDD'

# UART 初始化
ser = None
if ENABLE_UART:
    ser = uart.UART(UART_DEV, UART_BAUD)

# ---- 共享状态 (Web 推流用) --------------------------------------
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg_bytes = b''
        self.status = {"fps": 0.0, "pos_cm": 0.0}
        self.recording = False
        self.record_file = None
        self.record_path = ""
        self.record_start_time = ""
        self.record_frame_count = 0

    def update(self, jpeg_bytes, status_dict):
        with self.lock:
            self.jpeg_bytes = jpeg_bytes
            self.status.update(status_dict)

    def get_jpeg(self):
        with self.lock:
            return self.jpeg_bytes

    def get_status(self):
        with self.lock:
            return dict(self.status)

shared = SharedState()

# MJPEG 连接限制: 同一时间只允许一个, 防止刷新时线程堆积
_mjpeg_lock = threading.Lock()
_mjpeg_active = False

# ---- 摄像头 (YOLO 模型原生分辨率) -------------------------------
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
W, H = detector.input_width(), detector.input_height()
dis = display.Display()

p = comm.CommProtocol(buff_size = 1024)

# FPS 统计
last_fps_ticks = time.ticks_ms()
frame_count = 0
fps_val = 0.0

# 锁定区域
locked_corners = None  # (TL, TR, BR, BL)
locked_area = 0

# ---- 单球追踪器参数 --------------------------------------------
MATCH_DIST     = 80    # 匹配门限 (像素)
COAST_MAX      = 15    # 丢帧后保持帧数 (YOLO检测不稳定时延长)
CONFIRM_HITS   = 1     # 首帧确认 (最快响应)
PREDICT_AHEAD  = 1     # 前向预测帧数 (补偿YOLO NPU推理延迟)
BALL_OFFSET_X  = 3     # 红圆x偏移 (正=右移, 负=左移, 校准用)


def clamp_point(p, w, h):
    """钳制点到图像范围内"""
    return (max(0, min(w - 1, int(p[0]))),
            max(0, min(h - 1, int(p[1]))))


def order_corners(corners):
    """按 x 分左右，各组按 y 排上下 → [左上, 右上, 右下, 左下]"""
    pts = sorted(corners, key=lambda p: p[0])
    left  = sorted(pts[:2], key=lambda p: p[1])
    right = sorted(pts[2:], key=lambda p: p[1])
    return [left[0], right[0], right[1], left[1]]


def _point_to_warp(px, py, TL, TR, BR, BL, dst_w, dst_h):
    """将原图坐标 (px,py) 透视映射到 warp 图坐标 — 双线性插值"""
    # 在 px 处求上下边界的 y
    if abs(TR[0] - TL[0]) > 1:
        top_y = TL[1] + (TR[1] - TL[1]) * (px - TL[0]) / (TR[0] - TL[0])
    else:
        top_y = (TL[1] + TR[1]) / 2.0
    if abs(BR[0] - BL[0]) > 1:
        bot_y = BL[1] + (BR[1] - BL[1]) * (px - BL[0]) / (BR[0] - BL[0])
    else:
        bot_y = (BL[1] + BR[1]) / 2.0
    v = (py - top_y) / max(bot_y - top_y, 1.0)
    v = max(0.0, min(1.0, v))

    # 在 py 处求左右边界的 x
    if abs(BL[1] - TL[1]) > 1:
        left_x = TL[0] + (BL[0] - TL[0]) * (py - TL[1]) / (BL[1] - TL[1])
    else:
        left_x = (TL[0] + BL[0]) / 2.0
    if abs(BR[1] - TR[1]) > 1:
        right_x = TR[0] + (BR[0] - TR[0]) * (py - TR[1]) / (BR[1] - TR[1])
    else:
        right_x = (TR[0] + BR[0]) / 2.0
    u = (px - left_x) / max(right_x - left_x, 1.0)
    u = max(0.0, min(1.0, u))

    return u * (dst_w - 1), v * (dst_h - 1)


# ================================================================
# 单球轻量追踪器 (Kalman平滑 + 贪心匹配 + coast)
# ================================================================

class Kalman1D:
    """一维 Kalman [pos, vel] — 纯手算, 零依赖"""
    def __init__(self, pos, vel=0.0):
        self.pos = pos
        self.vel = vel
        self.p00 = 100.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 100.0

    def predict(self):
        self.pos += self.vel
        self.p00 += 2.0 * self.p01 + self.p11 + 1.0
        self.p01 += self.p11
        self.p10 = self.p01
        self.p11 += 5.0

    def update(self, z, R=25.0):
        s = self.p00 + R
        k0 = self.p00 / s
        k1 = self.p10 / s
        residual = z - self.pos
        self.pos += k0 * residual
        self.vel += k1 * residual
        self.p00 -= k0 * self.p00
        self.p01 -= k0 * self.p01
        self.p10 -= k1 * self.p00
        self.p11 -= k1 * self.p01


class BallTracker:
    """单球追踪: 两个1D Kalman + coast + 前向预测(防过冲)"""
    def __init__(self):
        self.kx = Kalman1D(0)
        self.ky = Kalman1D(0)
        self.w, self.h = 0.0, 0.0
        self.score = 0.0
        self.hits = 0
        self.misses = 0
        self.confirmed = False
        self.alive = False
        self.last_det_cx = 0.0   # 最新检测位置
        self.last_det_cy = 0.0

    def predict(self):
        self.kx.predict()
        self.ky.predict()

    def update(self, cx, cy, w, h, score):
        self.kx.update(cx)
        self.ky.update(cy)
        self.w  = 0.6 * w + 0.4 * self.w
        self.h  = 0.6 * h + 0.4 * self.h
        self.score = score
        self.hits += 1
        self.misses = 0
        self.last_det_cx = cx
        self.last_det_cy = cy
        if not self.alive:
            self.kx.pos = cx
            self.ky.pos = cy
            self.kx.vel = 0.0
            self.ky.vel = 0.0
            self.last_det_cx = cx
            self.last_det_cy = cy
            self.alive = True
        if self.hits >= CONFIRM_HITS:
            self.confirmed = True

    def mark_miss(self):
        self.misses += 1
        if self.misses > COAST_MAX:
            self.confirmed = False
            self.alive = False

    def output(self):
        """前向预测 + 钳制: 不超出最新检测位置半个球宽"""
        px = self.kx.pos + self.kx.vel * PREDICT_AHEAD
        py = self.ky.pos + self.ky.vel * PREDICT_AHEAD
        # 防过冲: 预测值不能超过检测位置 ± 球半径
        limit = max(self.w * 0.6, 5.0)   # 至少5px余量
        if self.kx.vel > 0:
            px = min(px, self.last_det_cx + limit)
        else:
            px = max(px, self.last_det_cx - limit)
        if self.ky.vel > 0:
            py = min(py, self.last_det_cy + limit)
        else:
            py = max(py, self.last_det_cy - limit)
        return px, py, self.w, self.h, self.score

    def vel_x(self):
        """Kalman x方向速度 (px/frame, 供级联PID内环用)"""
        return self.kx.vel


# ---- 录像功能 --------------------------------------------------
def start_recording():
    """开始录像，返回是否成功"""
    if shared.recording:
        return False
    try:
        os.makedirs(RECORD_DIR, exist_ok=True)
        # 文件名: rec_<boot_ms>.mjpeg
        ts = str(time.ticks_ms())
        fname = f"rec_{ts}.mjpeg"
        fpath = os.path.join(RECORD_DIR, fname)
        f = open(fpath, 'wb')
        # 文件头: magic MJPEG(5B) + version(1B) + frame_count(4B) + elapsed_ms(4B) = 14B
        f.write(b'MJPEG\x01\x00\x00\x00\x00\x00\x00\x00\x00')
        f.flush()
        shared.record_file = f
        shared.record_path = fpath
        shared.record_start_time = ts
        shared.record_frame_count = 0
        shared.recording = True
        print(f"[REC] Recording started: {fpath}")
        return True
    except Exception as e:
        print(f"[REC] Failed to start recording: {e}")
        return False


def stop_recording():
    """停止录像，更新文件头中的帧数，返回录像信息"""
    info = {"path": shared.record_path, "frames": 0, "size_kb": 0}
    if not shared.recording:
        return info
    shared.recording = False
    f = shared.record_file
    shared.record_file = None
    if f:
        try:
            # 回写帧数+时长到文件头 (偏移6: frame_count 4B, 偏移10: elapsed_ms 4B)
            elapsed = time.ticks_ms() - int(shared.record_start_time)
            f.seek(6)
            f.write(struct.pack('<II', shared.record_frame_count, elapsed))
            f.flush()
            fsize = f.tell()
            f.close()
            info = {"path": shared.record_path,
                     "frames": shared.record_frame_count,
                     "size_kb": fsize // 1024}
            print(f"[REC] Recording stopped: {shared.record_path} "
                  f"frames={shared.record_frame_count} elapsed={elapsed}ms size={fsize}B")
        except Exception as e:
            print(f"[REC] Error closing recording: {e}")
    return info


def write_recording_frame(jpeg_bytes):
    """写入一帧到录像文件 (4B LE 长度 + JPEG 数据)"""
    if not shared.recording or shared.record_file is None:
        return
    try:
        shared.record_file.write(
            struct.pack('<I', len(jpeg_bytes)) + jpeg_bytes)
        shared.record_file.flush()
        shared.record_frame_count += 1
    except Exception as e:
        print(f"[REC] Write error: {e}")
        stop_recording()

# 简单 URL 解码（替代 urllib.parse.unquote，兼容 MaixPy）
# ============================================================

def _url_decode(s):
    """URL 解码: %XX → 字符, + → 空格"""
    i = 0
    res = []
    while i < len(s):
        c = s[i]
        if c == '%' and i + 2 < len(s):
            try:
                res.append(chr(int(s[i+1:i+3], 16)))
                i += 3
                continue
            except Exception:
                pass
        elif c == '+':
            res.append(' ')
        else:
            res.append(c)
        i += 1
    return ''.join(res)

# HTTP 请求处理器（内置 http.server，零依赖）
# ============================================================

class StreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        path = self.path.split('?')[0]

        if path == '/':
            self._serve_html()
        elif path == '/video_feed':
            self._serve_mjpeg()
        elif path == '/api/status':
            self._serve_json()
        elif path == '/snapshot':
            self._serve_snapshot()
        elif path == '/api/record/start':
            self._record_start()
        elif path == '/api/record/stop':
            self._record_stop()
        elif path == '/api/recordings':
            self._list_recordings()
        elif path.startswith('/play/'):
            self._serve_recording(_url_decode(path[6:]))
        elif path.startswith('/api/recordings/') and path.endswith('/delete'):
            self._delete_recording(_url_decode(path[16:-7]))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')

    def _serve_html(self):
        body = INDEX_HTML.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _serve_json(self):
        status = shared.get_status()
        body = json.dumps(status).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _serve_snapshot(self):
        """返回当前实时 JPEG 帧（供浏览器录像抓帧使用）"""
        jpeg = shared.get_jpeg()
        if not jpeg:
            self.send_response(204)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', len(jpeg))
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_mjpeg(self):
        """MJPEG 流：同一时间只允许一个连接, 防止刷新时线程堆积崩溃"""
        global _mjpeg_active
        with _mjpeg_lock:
            if _mjpeg_active:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b'Too many streams')
                return
            _mjpeg_active = True

        try:
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()

            last_sent = None
            while True:
                jpeg = shared.get_jpeg()
                if jpeg and jpeg != last_sent:
                    last_sent = jpeg
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(jpeg)}\r\n'.encode())
                        self.wfile.write(b'\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError,
                            OSError):
                        break
                    time.sleep_ms(5)
                else:
                    time.sleep_ms(10)
        finally:
            with _mjpeg_lock:
                _mjpeg_active = False

    # ---- 录像控制 API ----

    def _record_start(self):
        """开始录像"""
        ok = start_recording()
        self._json_resp({"ok": ok, "recording": shared.recording,
                          "file": shared.record_path if ok else ""})

    def _record_stop(self):
        """停止录像"""
        info = stop_recording()
        self._json_resp({"ok": True, "file": info["path"],
                          "frames": info["frames"],
                          "size_kb": info["size_kb"]})

    def _list_recordings(self):
        """列出所有录像文件"""
        files = []
        try:
            os.makedirs(RECORD_DIR, exist_ok=True)
            for f in sorted(os.listdir(RECORD_DIR), reverse=True):
                if f.endswith('.mjpeg'):
                    fpath = os.path.join(RECORD_DIR, f)
                    size = os.path.getsize(fpath)
                    # 读取帧数 + 录制时长
                    fc = 0
                    el = 0
                    try:
                        with open(fpath, 'rb') as hdr:
                            hdr.read(6)
                            fc = struct.unpack('<I', hdr.read(4))[0]
                            el = struct.unpack('<I', hdr.read(4))[0]
                    except Exception:
                        fc = 0; el = 0
                    files.append({"name": f, "size_kb": size // 1024,
                                   "size_mb": size / (1024 * 1024),
                                   "frames": fc, "elapsed_ms": el})
        except Exception as e:
            print(f"[API] list_recordings error: {e}")
        self._json_resp({"recordings": files, "recording": shared.recording})

    def _serve_recording(self, filename):
        """播放录像：读取 .mjpeg 文件并以 MJPEG 流发送, 支持 ?start=N 跳转"""
        safe = os.path.basename(filename)
        if not safe.endswith('.mjpeg'):
            print(f"[PLAY] Invalid file type: {safe}")
            self.send_response(403)
            self.end_headers()
            self.wfile.write(b'Invalid file')
            return

        fpath = os.path.join(RECORD_DIR, safe)
        if not os.path.exists(fpath):
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Recording not found')
            return

        # 解析 ?start=N 查询参数
        start_frame = 0
        qs = self.path.split('?')
        if len(qs) > 1:
            for kv in qs[1].split('&'):
                if kv.startswith('start='):
                    try:
                        start_frame = int(kv[6:])
                    except Exception:
                        start_frame = 0

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        try:
            with open(fpath, 'rb') as f:
                # 读取头部: MJPEG(5) + version(1) + frame_count(4) + elapsed_ms(4) = 14 bytes
                header = f.read(14)
                hdr_fc  = struct.unpack('<I', header[6:10])[0]
                hdr_ems = struct.unpack('<I', header[10:14])[0]
                # 按实际录制帧率回放 (避免进度条和画面不同步)
                if hdr_fc > 0 and hdr_ems > 0:
                    frame_interval = int(hdr_ems / hdr_fc)
                    frame_interval = max(20, min(frame_interval, 200))
                else:
                    frame_interval = 50
                # 跳过前 N 帧
                skipped = 0
                while skipped < start_frame:
                    size_bytes = f.read(4)
                    if len(size_bytes) < 4:
                        break
                    frame_size = struct.unpack('<I', size_bytes)[0]
                    f.read(frame_size)
                    skipped += 1
                while True:
                    size_bytes = f.read(4)
                    if len(size_bytes) < 4:
                        break
                    frame_size = struct.unpack('<I', size_bytes)[0]
                    jpeg = f.read(frame_size)
                    if len(jpeg) < frame_size:
                        break
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {frame_size}\r\n'.encode())
                        self.wfile.write(b'\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        time.sleep_ms(frame_interval)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
        except Exception as e:
            print(f"[PLAY] Stream error for {safe}: {type(e).__name__}: {e}")

    def _delete_recording(self, filename):
        """删除录像"""
        safe = os.path.basename(filename)
        if not safe.endswith('.mjpeg'):
            self._json_resp({"ok": False, "error": "invalid file"})
            return
        fpath = os.path.join(RECORD_DIR, safe)
        try:
            if os.path.exists(fpath):
                os.remove(fpath)
            self._json_resp({"ok": True})
        except Exception as e:
            self._json_resp({"ok": False, "error": str(e)})

    def _json_resp(self, data):
        body = json.dumps(data).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(body))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass


def get_local_ip():
    """获取本机局域网 IP"""
    return '172.30.231.179'


def start_web_server(port):
    """在守护线程中启动 HTTP 服务器"""
    if ThreadingHTTPServer is not None:
        server = ThreadingHTTPServer(('0.0.0.0', port), StreamHandler)
        print(f"[WEB] Using multi-threaded HTTP server")
    else:
        server = HTTPServer(('0.0.0.0', port), StreamHandler)
        print(f"[WEB] Warning: single-threaded server")
    ip = get_local_ip()
    print(f"[WEB] ========================================")
    print(f"[WEB]  HTTP server started on port {port}")
    if ip:
        print(f"[WEB]  Open in browser:")
        print(f"[WEB]    http://{ip}:{port}")
    print(f"[WEB] ========================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

# ---- Web 服务器启动 (守护线程) ------------------------------------
if WEB_ENABLE:
    web_thread = threading.Thread(
        target=start_web_server, args=(WEB_PORT,),
        daemon=True, name="WebServer"
    )
    web_thread.start()
    print(f"[WEB] Web server thread started (port {WEB_PORT})")

# ---- HTML 页面 ------------------------------------------------
INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>钢球检测 - Browser Record</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0f0f1a;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:20px}
h2{color:#00d4ff;margin-bottom:4px;font-size:22px}
.sub{font-size:13px;color:#666;margin-bottom:20px}
.container{max-width:880px;width:100%}
.section{background:#1a1a2e;border-radius:10px;padding:16px;margin-bottom:14px;border:1px solid #252545}
.section-header{display:flex;align-items:center;gap:8px;margin-bottom:10px;color:#00d4ff;font-size:14px;font-weight:600}
.section-header .icon{font-size:16px}
/* MJPEG 实时画面 */
.stream-img{border-radius:6px;overflow:hidden;text-align:center}
.stream-img img{width:100%;max-width:640px;border-radius:6px;display:block;margin:0 auto}
/* 录像控制 */
.rec-row{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.btn-rec{background:#e74c3c;color:#fff;border:none;padding:10px 22px;border-radius:6px;cursor:pointer;font-size:14px;font-weight:600;transition:background .2s}
.btn-rec:hover{background:#c0392b}
.btn-rec.stop{background:#555}
.btn-rec.stop:hover{background:#666}
/* 回放 */
.playback-placeholder{background:#0a0a1a;border-radius:6px;width:100%;max-width:640px;height:300px;display:flex;align-items:center;justify-content:center;color:#444;font-size:14px;margin:0 auto}
/* 按钮 */
.btn-sm{background:#2a2a4a;color:#aaa;border:1px solid #3a3a5a;padding:5px 12px;border-radius:4px;cursor:pointer;font-size:12px;transition:all .2s}
.btn-sm:hover{background:#35355a;color:#ccc}
.btn-play{background:#2980b9;color:#fff;border:none;padding:4px 12px;border-radius:4px;cursor:pointer;font-size:11px;transition:background .2s}
.btn-play:hover{background:#3498db}
.btn-del{background:#c0392b;color:#fff;border:none;padding:4px 12px;border-radius:4px;cursor:pointer;font-size:11px;transition:background .2s}
.btn-del:hover{background:#e74c3c}
.btn-stop-play{background:#e67e22;color:#fff;border:none;padding:4px 12px;border-radius:4px;cursor:pointer;font-size:11px}
.btn-dl{background:#27ae60;color:#fff;border:none;padding:4px 12px;border-radius:4px;cursor:pointer;font-size:11px;transition:background .2s}
.btn-dl:hover{background:#2ecc71}
/* 录像列表 */
.rec-list{max-height:260px;overflow-y:auto}
.rec-item{display:flex;align-items:center;padding:6px 8px;border-bottom:1px solid #1e1e3a;flex-wrap:wrap;gap:6px;border-radius:4px;transition:background .15s}
.rec-item:hover{background:#1e1e3a}
.rec-name{flex:1;color:#ccc;font-size:13px;min-width:100px}
.rec-meta{color:#666;font-size:11px;white-space:nowrap}
.empty-hint{color:#555;font-size:13px;text-align:center;padding:20px}
</style>
</head>
<body>
<div class="container">
    <h2>&#127936; 钢球检测</h2>
    <p class="sub">MaixCAM2 NPU &middot; 实时推流 &middot; 浏览器录像</p>

    <!-- ====== 实时画面 ====== -->
    <div class="section">
        <div class="section-header"><span class="icon">&#128247;</span> 实时画面</div>
        <div class="stream-img">
            <img id="streamLive" src="/video_feed" alt="实时视频流"
                 onerror="setTimeout(function(){this.src='/video_feed?t='+Date.now();},500)">
        </div>
    </div>

    <!-- ====== 浏览器端录像 ====== -->
    <div class="section">
        <div class="section-header"><span class="icon">&#128190;</span> 浏览器端录像 (不占设备资源)</div>
        <div class="rec-row">
            <button id="browserRecBtn" class="btn-rec" onclick="toggleBrowserRecord()">&#9679; 开始录制</button>
            <span id="browserRecStatus" style="color:#888;font-size:13px;"></span>
            <span id="browserRecSize" style="color:#666;font-size:12px;margin-left:8px;"></span>
        </div>
    </div>

    <!-- ====== 录像回放 ====== -->
    <div class="section">
        <div class="section-header">
            <span class="icon">&#127910;</span> 录像回放
            <button id="btnStopPlay" class="btn-stop-play" onclick="stopPlayback()" style="display:none;">&#9632; 停止回放</button>
            <span id="playbackTime" style="color:#00d4ff;font-size:13px;margin-left:8px;display:none;"></span>
        </div>
        <div id="playbackContainer">
            <div id="playbackPlaceholder" class="playback-placeholder">&#9654; 点击下方录像「播放」回放</div>
            <canvas id="playbackCanvas" style="display:none;width:100%;max-width:640px;border-radius:6px;margin:0 auto;"></canvas>
        </div>
        <div id="playbackBar" style="display:none;margin-top:8px;">
            <input type="range" id="seekBar" min="0" max="100" value="0" step="1"
                   style="width:100%;max-width:640px;display:block;margin:0 auto;accent-color:#e74c3c;"
                   oninput="onSeek(this.value)">
            <div style="display:flex;justify-content:space-between;max-width:640px;margin:2px auto 0;font-size:11px;color:#666;">
                <span id="seekLabelStart">00:00</span><span id="seekLabelEnd">00:00</span>
            </div>
        </div>
    </div>

    <!-- ====== 录像列表 ====== -->
    <div class="section">
        <div class="section-header">
            <span class="icon">&#128193;</span> 录像列表 (浏览器内存)
            <button class="btn-sm" onclick="renderRecordingList()">&#8635; 刷新</button>
        </div>
        <div id="recList" class="rec-list">
            <div class="empty-hint">暂无录像，点击「开始录制」进行录制</div>
        </div>
    </div>

    <!-- 隐藏canvas用于抓帧 -->
    <canvas id="captureCanvas" style="display:none;"></canvas>
</div>

<script>//<![CDATA[
var gBrowserRecs=[];          // {name, frames:[blob], fps, startMs, elapsedMs, sizeBytes}
var gIsRecording=false;
var gRecTimer=null,gRecStartMs=0,gRecFrameCount=0,gRecFrames=[];
var gRecFps=30;              // 录制帧率, 实际帧数取决于推流速率
var gPlayTimer=null,gPlayIdx=0,gPlayFrames=null,gPlayFps=30;

function fmtDuration(sec){
    var m=Math.floor(sec/60);
    var s=Math.floor(sec%60);
    return (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}
function fmtSize(bytes){
    if(bytes<1024)return bytes+'B';
    if(bytes<1048576)return (bytes/1024).toFixed(1)+'KB';
    return (bytes/1048576).toFixed(1)+'MB';
}

// ---- 浏览器端录像 ----
function toggleBrowserRecord(){
    if(!gIsRecording){
        gRecFrames=[];
        gRecFrameCount=0;
        gRecStartMs=Date.now();
        gIsRecording=true;
        var btn=document.getElementById('browserRecBtn');
        btn.innerHTML='&#9632; 停止录制';
        btn.className='btn-rec stop';
        gRecTimer=setInterval(captureFrame, 1000/gRecFps);
        updateRecStatus();
    }else{
        stopBrowserRecord();
    }
}

function captureFrame(){
    var liveImg=document.getElementById('streamLive');
    var canvas=document.getElementById('captureCanvas');
    if(!liveImg||!liveImg.complete||!liveImg.naturalWidth)return;
    canvas.width=liveImg.naturalWidth;
    canvas.height=liveImg.naturalHeight;
    var ctx=canvas.getContext('2d');
    ctx.drawImage(liveImg,0,0);
    canvas.toBlob(function(blob){
        if(blob){
            gRecFrames.push(blob);
            gRecFrameCount=gRecFrames.length;
            updateRecStatus();
        }
    },'image/jpeg',0.85);
}

function updateRecStatus(){
    var el=(Date.now()-gRecStartMs)/1000;
    var totalSize=gRecFrames.reduce(function(s,f){return s+f.size;},0);
    document.getElementById('browserRecStatus').innerHTML='&#9202; '+fmtDuration(el)+' | '+gRecFrameCount+'帧';
    document.getElementById('browserRecSize').innerHTML=fmtSize(totalSize);
}

function stopBrowserRecord(){
    gIsRecording=false;
    if(gRecTimer){clearInterval(gRecTimer);gRecTimer=null;}
    var el=(Date.now()-gRecStartMs)/1000;
    var totalSize=gRecFrames.reduce(function(s,f){return s+f.size;},0);
    var name='rec_'+new Date().toISOString().replace(/[:.]/g,'-').slice(0,19)+'.mjpeg';
    gBrowserRecs.unshift({
        name:name,
        frames:gRecFrames.slice(),
        fps:gRecFps,
        startMs:gRecStartMs,
        elapsedMs:Date.now()-gRecStartMs,
        sizeBytes:totalSize,
        frameCount:gRecFrameCount
    });
    var btn=document.getElementById('browserRecBtn');
    btn.innerHTML='&#9679; 开始录制';
    btn.className='btn-rec';
    document.getElementById('browserRecStatus').innerHTML='已保存 '+fmtDuration(el)+' | '+gRecFrameCount+'帧';
    document.getElementById('browserRecSize').innerHTML=fmtSize(totalSize);
    renderRecordingList();
}

// ---- 回放 ----
function playBrowserRecord(idx){
    stopPlayback();
    var rec=gBrowserRecs[idx];
    if(!rec||!rec.frames.length)return;
    gPlayFrames=rec.frames;
    gPlayIdx=0;
    gPlayFps=rec.fps;

    var canvas=document.getElementById('playbackCanvas');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    var tm=document.getElementById('playbackTime');
    var bar=document.getElementById('playbackBar');
    var seek=document.getElementById('seekBar');
    canvas.style.display='block';
    ph.style.display='none';
    btn.style.display='inline-block';
    tm.style.display='inline';
    bar.style.display='block';
    seek.max=gPlayFrames.length-1;
    seek.value=0;
    document.getElementById('seekLabelStart').textContent='00:00';
    document.getElementById('seekLabelEnd').textContent=fmtDuration(gPlayFrames.length/gPlayFps);

    showPlayFrame();
    gPlayTimer=setInterval(function(){
        gPlayIdx++;
        if(gPlayIdx>=gPlayFrames.length){gPlayIdx=0;}
        seek.value=gPlayIdx;
        showPlayFrame();
    },1000/gPlayFps);
    renderRecordingList();
}

function showPlayFrame(){
    if(!gPlayFrames||gPlayIdx>=gPlayFrames.length)return;
    var blob=gPlayFrames[gPlayIdx];
    var url=URL.createObjectURL(blob);
    var canvas=document.getElementById('playbackCanvas');
    var img=new Image();
    img.onload=function(){
        canvas.width=img.naturalWidth;
        canvas.height=img.naturalHeight;
        canvas.getContext('2d').drawImage(img,0,0);
        URL.revokeObjectURL(url);
        document.getElementById('playbackTime').textContent=
            fmtDuration(gPlayIdx/gPlayFps)+' / '+fmtDuration(gPlayFrames.length/gPlayFps);
    };
    img.src=url;
}

function stopPlayback(){
    if(gPlayTimer){clearInterval(gPlayTimer);gPlayTimer=null;}
    gPlayFrames=null;gPlayIdx=0;
    document.getElementById('playbackCanvas').style.display='none';
    document.getElementById('playbackPlaceholder').style.display='flex';
    document.getElementById('btnStopPlay').style.display='none';
    document.getElementById('playbackTime').style.display='none';
    document.getElementById('playbackBar').style.display='none';
    var ctx=document.getElementById('playbackCanvas').getContext('2d');
    ctx.clearRect(0,0,document.getElementById('playbackCanvas').width,document.getElementById('playbackCanvas').height);
    renderRecordingList();
}

function onSeek(val){
    var f=parseInt(val);
    gPlayIdx=f;
    showPlayFrame();
    document.getElementById('playbackTime').textContent=fmtDuration(f/gPlayFps);
    document.getElementById('seekLabelStart').textContent=fmtDuration(f/gPlayFps);
}

// ---- 下载 ----
function downloadBrowserRecord(idx){
    var rec=gBrowserRecs[idx];
    if(!rec)return;
    // 组装 .mjpeg: 每帧 4B LE长度 + JPEG数据
    var parts=[];
    var header=new ArrayBuffer(14);
    var dv=new DataView(header);
    dv.setUint8(0,77);dv.setUint8(1,74);dv.setUint8(2,80);dv.setUint8(3,69);dv.setUint8(4,71); // MJPEG
    dv.setUint8(5,1);  // version
    dv.setUint32(6,rec.frameCount,true);
    dv.setUint32(10,rec.elapsedMs,true);
    parts.push(new Uint8Array(header));
    for(var i=0;i<rec.frames.length;i++){
        var lenBuf=new ArrayBuffer(4);
        new DataView(lenBuf).setUint32(0,rec.frames[i].size,true);
        parts.push(new Uint8Array(lenBuf));
    }
    // 需要异步读取所有blob
    var allParts=parts.slice();
    var remaining=rec.frames.length;
    for(var i=0;i<rec.frames.length;i++){
        (function(idx){
            var reader=new FileReader();
            reader.onload=function(e){
                allParts[idx+1]=new Uint8Array(e.target.result);
                remaining--;
                if(remaining===0){
                    // 计算总大小
                    var totalLen=allParts.reduce(function(s,p){return s+p.length;},0);
                    var merged=new Uint8Array(totalLen);
                    var off=0;
                    for(var j=0;j<allParts.length;j++){
                        merged.set(allParts[j],off);
                        off+=allParts[j].length;
                    }
                    var blob=new Blob([merged],{type:'application/octet-stream'});
                    var a=document.createElement('a');
                    a.href=URL.createObjectURL(blob);
                    a.download=rec.name;
                    a.click();
                    URL.revokeObjectURL(a.href);
                }
            };
            reader.readAsArrayBuffer(rec.frames[idx]);
        })(i);
    }
}

function deleteBrowserRecord(idx){
    if(!confirm('删除 '+gBrowserRecs[idx].name+' ?'))return;
    if(gPlayFrames===gBrowserRecs[idx].frames)stopPlayback();
    gBrowserRecs.splice(idx,1);
    renderRecordingList();
}

// ---- 渲染录像列表 ----
function renderRecordingList(){
    var list=document.getElementById('recList');
    if(gBrowserRecs.length===0){
        list.innerHTML='<div class="empty-hint">暂无录像，点击「开始录制」进行录制</div>';
        return;
    }
    var h='';
    for(var i=0;i<gBrowserRecs.length;i++){
        var r=gBrowserRecs[i];
        var isPlaying=(gPlayFrames===r.frames);
        h+='<div class="rec-item">'+
            '<span class="rec-name" style="color:#00d4ff;">'+(isPlaying?'&#9654; ':'')+r.name+'</span>'+
            '<span class="rec-meta">'+fmtDuration(r.elapsedMs/1000)+' | '+r.frameCount+'帧 | '+fmtSize(r.sizeBytes)+'</span>'+
            '<button class="btn-play" onclick="playBrowserRecord('+i+')">播放</button>'+
            '<button class="btn-dl" onclick="downloadBrowserRecord('+i+')">下载</button>'+
            '<button class="btn-del" onclick="deleteBrowserRecord('+i+')">删除</button>'+
            '</div>';
    }
    list.innerHTML=h;
}

// ---- 启动 ----
renderRecordingList();
//]]></script>
</body>
</html>
"""

print(f"[INFO] YOLO input: {W}x{H}")
print("[INFO] LED fill light: ON")
if ENABLE_UART:
    print(f"[INFO] UART: {UART_DEV}@{UART_BAUD}")
else:
    print("[INFO] UART: OFF")
print("[INFO] Waiting for green tube detection...")

# ---- 调试日志 ----
DEBUG_LOG = "/root/debug.log"

def dbg(msg):
    """追加一行带时间戳的调试日志, 立即刷盘"""
    try:
        with open(DEBUG_LOG, "a") as f:
            f.write(f"[{time.ticks_ms()}] {msg}\n")
            f.flush()
    except Exception:
        pass

dbg("=== Program start ===")

# 单球追踪器
tracker = BallTracker()

# ================================================================
# 主循环
# ================================================================
while not app.need_exit():
    try:
        img = cam.read()
    except Exception as _e:
        dbg(f"cam.read CRASH: {_e}")
        break
    frame_count += 1
    # 消除 Pyright possibly-unbound 误报 (运行时始终会被覆盖)
    servo_angle = BAL_CENTER; p_term = d_term = i_term = _bias = 0.0; _setpoint = 0.0
    tcx = tcy = tw = th = 0.0; tscore = 0.0
    TL = TR = BR = BL = (0, 0); dst_h = mid_y = 0
    img_warped = img

    # ---- 阶段1: 绿色管道检测 (仅一次) ----
    if locked_corners is None:
        blobs = img.find_blobs([GREEN_THRESHOLD],
                               area_threshold=MIN_AREA,
                               pixels_threshold=MIN_AREA,
                               merge=MERGE,
                               margin=MARGIN,
                               x_stride=2, y_stride=2)
        if blobs:
            b = max(blobs, key=lambda b: b.area())
            raw_TL, raw_TR, raw_BR, raw_BL = order_corners(b.mini_corners())
            locked_corners = (raw_TL, raw_TR, raw_BR, raw_BL)
            locked_area = b.area()
            print(f"[INFO] Green tube locked! area={locked_area}")

    # ---- YOLO 推理 ----
    raw_objs = detector.detect(img, conf_th=0.4, iou_th=0.45)
    best_det = None
    if len(raw_objs) > 0:
        best_det = max(raw_objs, key=lambda o: o.score)

    has_track = False
    ball_cm = 0.0

    # ---- 透视矫正参数 (算一次, 追踪 + 显示共用) ----
    if locked_corners is not None:
        servo_angle = BAL_CENTER  # 默认值 (舵机关闭或未追踪时使用)
        TL, TR, BR, BL = locked_corners
        TL = clamp_point(TL, W, H)
        TR = clamp_point(TR, W, H)
        BR = clamp_point(BR, W, H)
        BL = clamp_point(BL, W, H)
        src_w = ((TR[0] - TL[0]) + (BR[0] - BL[0])) / 2.0
        src_h = ((BL[1] - TL[1]) + (BR[1] - TR[1])) / 2.0
        scale = (W - 1) / max(src_w, 1)
        dst_h = int(src_h * scale)
        dst_h = max(dst_h, 2)
        dst_h = dst_h // 2 * 2
        src_flat = [TL[0], TL[1], TR[0], TR[1], BR[0], BR[1], BL[0], BL[1]]
        dst_flat = [0, 0, W - 1, 0, W - 1, dst_h - 1, 0, dst_h - 1]
        img_warped = img.perspective(src_flat, dst_flat, W, dst_h)

        # 三条虚线: 中心 + ±5cm (黄色)
        mid_y = dst_h // 2
        cx_0  = W // 2
        cx_p5 = cx_0 + int(W * 5.0 / PIPE_LENGTH)
        cx_n5 = cx_0 - int(W * 5.0 / PIPE_LENGTH)
        for cy in range(0, dst_h, 10):
            ye = min(cy + 5, dst_h)
            img_warped.draw_line(cx_n5, cy, cx_n5, ye, image.COLOR_YELLOW, 1)
            img_warped.draw_line(cx_0,  cy, cx_0,  ye, image.COLOR_YELLOW, 1)
            img_warped.draw_line(cx_p5, cy, cx_p5, ye, image.COLOR_YELLOW, 1)
        # 水平中心实线 (红色)
        img_warped.draw_line(0, mid_y, W - 1, mid_y, image.COLOR_RED, 1)
        # 管道端点虚线 (青色, ±12.5cm 各内缩 PIPE_INSET_LEFT + PIPE_INSET_RIGHT)
        cx_left_end  = PIPE_INSET_LEFT
        cx_right_end = W - 1 - PIPE_INSET_RIGHT
        for cy in range(0, dst_h, 10):
            ye = min(cy + 5, dst_h)
            img_warped.draw_line(cx_left_end,  cy, cx_left_end,  ye, image.COLOR_BLUE, 1)
            img_warped.draw_line(cx_right_end, cy, cx_right_end, ye, image.COLOR_BLUE, 1)

    # ---- 追踪只在绿框锁定后运行 ----
    if locked_corners is not None:
        tracker.predict()
        tracker.mark_miss()

        if best_det:
            dcx = best_det.x + best_det.w // 2
            dcy = best_det.y + best_det.h // 2
            if tracker.confirmed:
                match_x, match_y = tracker.kx.pos, tracker.ky.pos
            else:
                match_x, match_y = dcx, dcy
            d2 = (dcx - match_x) ** 2 + (dcy - match_y) ** 2
            if d2 < MATCH_DIST * MATCH_DIST:
                tracker.update(dcx, dcy, best_det.w, best_det.h, best_det.score)

        has_track = tracker.confirmed

        if has_track:
            tcx, tcy, tw, th, tscore = tracker.output()
            bw_x, _ = _point_to_warp(tcx, tcy, TL, TR, BR, BL, W, dst_h)
            bw_x += BALL_OFFSET_X
            effective_w = W - (PIPE_INSET_LEFT + PIPE_INSET_RIGHT)
            ball_cm = (bw_x - W / 2.0) * PIPE_LENGTH / max(effective_w, 1)

            if report_on:
                class _T: pass
                tobj = _T()
                tobj.x = int(tcx - tw // 2)
                tobj.y = int(tcy - th // 2)
                tobj.w = int(tw)
                tobj.h = int(th)
                o = _T()
                o.x, o.y = tobj.x, tobj.y
                o.w, o.h = tobj.w, tobj.h
                o.class_id = 0
                o.score = tscore
                body = encode_objs([o])
                p.report(APP_CMD_DETECT_RES, body)

    # ---- UART 发送 (基于 warp 坐标的 cm) ----
    if ser is not None and locked_corners is not None:
        packet = build_packet(ball_cm)
        ser.write(packet)

    # ---- 舵机控制 ----
    if SERVO_ENABLE and locked_corners is not None:
        p_term = d_term = i_term = 0.0; _pid_vel = 0.0
        if CURRENT_MODE == 2:
            # === Mode 2 独立简洁PID: 纯PD+bias, 无I, 宽阈值切换 ===
            # 序列: state1(归零) → state2(+5) → state3(-5) → state4(保持)
            M2_KP = 6.0; M2_KD = 3.0; M2_DB = 0.3   # 宽死区 for state 1-3
            M2_DB_HOLD = 0.1  # 精准死区 for state 4
            now_ms = time.ticks_ms()
            if _pid_last_ms == 0: dt = 0.016
            else:
                dt = (now_ms - _pid_last_ms) / 1000.0
                if dt <= 0 or dt > 1.0: dt = 0.016
            _pid_last_ms = now_ms
            if dt > 0.001:
                _pid_vel = tracker.vel_x() * (PIPE_LENGTH / max(W - (PIPE_INSET_LEFT + PIPE_INSET_RIGHT), 1)) / dt

            if _m2_state == 0:
                # 空闲: 简单PD回中, 不加bias
                _setpoint = 0.0
                if has_track:
                    error = ball_cm; abs_err = abs(error)
                    s = 1.0 if error > 0 else -1.0
                    p_term = M2_KP * (0.5 if abs_err > 3.0 else 1.0) * (error - s * M2_DB) if abs_err >= M2_DB else 0.0
                    d_term = max(-120.0, min(120.0, M2_KD * _pid_vel))
                    i_term = 0.0; _bias = 0.0
                else:
                    servo_angle = _pid_last_out; p_term = d_term = i_term = _bias = 0.0
            elif has_track:
                # 确定目标
                if _m2_state == 1:   _m2_set = 0.0
                elif _m2_state == 2: _m2_set = 4.5
                elif _m2_state == 3: _m2_set = -5.0
                else:                _m2_set = -5.0
                _setpoint = _m2_set
                error = ball_cm - _setpoint; abs_err = abs(error)

                # 状态转换 (宽阈值0.5cm, 球接近就切, 不等完全静止)
                if _m2_state == 1 and abs_err < 0.7 and abs(_pid_vel) < 5.0:
                    _m2_state = 2; _bias_ramp = 0.0
                elif _m2_state == 2 and abs_err < 0.5:
                    _m2_state = 3; _bias_ramp = 0.0
                elif _m2_state == 3 and abs_err < 0.5:
                    _m2_state = 4; _bias_ramp = 0.0; _pid_integral = 0.0
                    _m2_timer_ms = now_ms - _m2_timer_start

                # 计时
                if 1 <= _m2_state <= 3:
                    _m2_timer_ms = now_ms - _m2_timer_start

                # 纯PD + bias前馈 (state 4用更紧的死区)
                s = 1.0 if error > 0 else -1.0
                _db = M2_DB_HOLD if _m2_state == 4 else M2_DB
                p_term = M2_KP * (0.5 if abs_err > 3.0 else 1.0) * (error - s * _db) if abs_err >= _db else 0.0
                d_term = max(-120.0, min(120.0, M2_KD * _pid_vel))

                # State 4 精准停靠: 加速I积分消除稳态误差
                if _m2_state == 4:
                    if error * _pid_integral < 0: _pid_integral = 0
                    _pid_integral += error * dt * 3.0
                    _pid_integral = max(-15.0, min(15.0, _pid_integral))
                    i_term = _pid_integral
                else:
                    i_term = 0.0  # State 1-3 不用I

                # bias: 固定前馈, 带ramp缓启动
                _bias_full = -(BAL_SLOPE_POS if _setpoint > 0 else BAL_SLOPE_NEG) * _setpoint
                _bias = _bias_full
                if _bias_ramp < 1.0:
                    _bias_ramp = min(1.0, _bias_ramp + 0.05)
                _bias *= _bias_ramp
            else:
                servo_angle = _pid_last_out; p_term = d_term = i_term = _bias = 0.0

            # 输出合成
            if has_track or _m2_state != 0:
                _m2_trim = 10.0 if _m2_state == 0 else 0.0  # state0居中时加偏置
                raw_out = (BAL_CENTER + _m2_trim) + p_term + d_term + i_term + _bias
                max_step = RATE * dt
                delta = raw_out - _pid_last_out
                if abs(delta) > max_step:
                    raw_out = _pid_last_out + max_step if delta > 0 else _pid_last_out - max_step
                servo_angle = max(BAL_MIN_ANGLE, min(BAL_MAX_ANGLE, raw_out))
                _pid_last_out = servo_angle
        elif (CURRENT_MODE == 1 or CURRENT_MODE == 3) and has_track:
            now_ms = time.ticks_ms()
            if _pid_last_ms == 0:
                dt = 0.016
            else:
                dt = (now_ms - _pid_last_ms) / 1000.0
                if dt <= 0 or dt > 1.0:
                    dt = 0.016
            _pid_last_ms = now_ms

            # Mode 3: 误差 = 球位置 - 目标点; Mode 1: 目标=中心(0)
            _setpoint = _target_cm if (CURRENT_MODE == 3 and _target_touched) else 0.0
            error = ball_cm - _setpoint
            abs_err = abs(error)

            # Kalman速度 (零滞后)
            if dt > 0.001:
                _pid_vel = tracker.vel_x() * (PIPE_LENGTH / max(W - (PIPE_INSET_LEFT + PIPE_INSET_RIGHT), 1)) / dt

            # === 单套 PID (v7: 预测性刹车 + I方向感知) ===
            # 方向判断
            moving_to_center = (error > 0 and _pid_vel < 0) or (error < 0 and _pid_vel > 0)
            moving_away = (error > 0 and _pid_vel > 0) or (error < 0 and _pid_vel < 0)

            # --- P项: 距离缩放 + 预测性减益 ---
            s = 1.0 if error > 0 else -1.0
            _ps = 0.5 if abs_err > 3.0 else 1.0  # 远区半P
            if moving_to_center and abs(_pid_vel) > 5.0:
                _ps *= 0.35  # 高速冲目标: 大幅减P
            elif moving_to_center and abs(_pid_vel) > 2.0:
                _ps *= 0.6   # 中速接近: 适度减P
            p_term = KP * _ps * (error - s * DB) if abs_err >= DB else 0.0

            # --- D项: 均匀阻尼 + 限幅防饱和 ---
            _kd_scale = 1.0 + 0.2 * max(0.0, 1.0 - abs_err)  # 近中心最多1.2x, 远区1.0x
            d_term = KD * _kd_scale * _pid_vel
            if d_term > 100.0: d_term = 100.0
            elif d_term < -100.0: d_term = -100.0

            # --- I项: 方向感知冻结 + 卡球加速 ---
            if error * _pid_integral < 0:
                _pid_integral = 0  # 过零点清I残值

            is_stuck = (abs_err > 1.0 and abs(_pid_vel) < 1.5
                        and not moving_to_center)  # 有偏差但不动且不往中心: 卡住了

            if is_stuck:
                # 卡球: 快速积I突破摩擦力
                _pid_integral += error * dt * 8.0
                _pid_integral = max(-12.0, min(12.0, _pid_integral))
            elif moving_to_center and abs(_pid_vel) > 3.0:
                # 球有动量冲目标: 冻结I
                pass
            elif abs_err < 0.15 and abs(_pid_vel) < 0.5:
                # 死区: 衰减I
                _pid_integral *= 0.8
            else:
                # 积分消除稳态误差, 大误差时减速防饱和
                _i_rate = 2.0
                if abs_err > 5.0:
                    _i_rate *= 0.3
                _pid_integral += error * dt * _i_rate
                _pid_integral = max(-12.0, min(12.0, _pid_integral))
            i_term = _pid_integral

            # === 输出合成 (含坡度前馈) ===
            _bias_full = -(BAL_SLOPE_POS if _setpoint > 0 else BAL_SLOPE_NEG) * _setpoint
            if abs_err > 5.0:
                _bias = _bias_full * max(0.15, 5.0 / abs_err)
            else:
                _bias = _bias_full
            # bias缓启动: 目标激活后从0线性增加到全量, 防超调
            if _bias_ramp < 1.0:
                _bias_ramp = min(1.0, _bias_ramp + 0.05)
            _bias *= _bias_ramp
            _m3_centering = (CURRENT_MODE == 3 and not _target_touched)
            _trim = 10.0 if (CURRENT_MODE == 1 or _m3_centering) else 0.0  # 居中时加偏置
            raw_out = (BAL_CENTER + _trim) + p_term + d_term + i_term + _bias
            max_step = RATE * dt
            delta = raw_out - _pid_last_out
            if abs(delta) > max_step:
                raw_out = _pid_last_out + max_step if delta > 0 else _pid_last_out - max_step
            servo_angle = max(BAL_MIN_ANGLE, min(BAL_MAX_ANGLE, raw_out))
            _pid_last_out = servo_angle
        else:
            servo_angle = _pid_last_out
            _pid_integral *= 0.8

        duty = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * servo_angle / 180.0
        servo.duty(duty)

    # ---- 显示 ----
    if locked_corners is not None:
        # 球心圆点 (红色)
        if has_track:
            bw_x, _ = _point_to_warp(tcx, tcy, TL, TR, BR, BL, W, dst_h)
            img_warped.draw_circle(int(bw_x + BALL_OFFSET_X), mid_y, 6, image.COLOR_RED, -1)

        # Mode 3 目标点 (绿色实心圆 + 外环)
        if CURRENT_MODE == 3 and _target_touched:
            _eff_w = W - (PIPE_INSET_LEFT + PIPE_INSET_RIGHT)
            _tgt_x = int(W / 2.0 + _target_cm * _eff_w / PIPE_LENGTH)
            img_warped.draw_circle(_tgt_x, mid_y, 8, image.COLOR_GREEN, -1)
            img_warped.draw_circle(_tgt_x, mid_y, 11, image.COLOR_GREEN, 2)

        # 在原图上画追踪框
        if has_track:
            bx = int(tcx - tw // 2)
            by = int(tcy - th // 2)
            img.draw_rect(bx, by, int(tw), int(th),
                          color=image.COLOR_GREEN, thickness=2)
            img.draw_string(bx, max(0, by - 14),
                            f"sb:{tscore:.2f}", color=image.COLOR_GREEN)

        if CURRENT_MODE == 3 and _target_touched:
            img.draw_string(4, 4,  f"F:{fps_val:.0f} cm:{ball_cm:+.1f} tgt:{_target_cm:+.1f} ang:{servo_angle:.0f} {'T' if has_track else '!'}", image.COLOR_WHITE)
        else:
            img.draw_string(4, 4,  f"F:{fps_val:.0f} cm:{ball_cm:+.1f} ang:{servo_angle:.0f} {'T' if has_track else '!'}", image.COLOR_WHITE)

        # Mode 3 微调控件 (原图左下角, 按下时绿色高亮)
        if CURRENT_MODE == 3:
            _FBTN_H = 44; _FBTN_Y = H - 52
            _labels  = ['<<', '<', '', '>', '>>', 'Set']
            _actions = ['--', '-', '', '+', '++', 'set']
            _widths  = [48, 36, 76, 36, 48, 76]
            _x0 = 5; _gap = 4
            _fine_btns.clear()
            for i in range(6):
                bx = _x0; bw = _widths[i]
                act = _actions[i]
                if act == '':
                    val = _target_cm if _target_touched else 0.0
                    img.draw_rect(bx, _FBTN_Y, bw, _FBTN_H, image.COLOR_WHITE, -1)
                    img.draw_string(bx+5, _FBTN_Y+10, f"{val:+.1f}", image.COLOR_BLACK)
                else:
                    color = image.COLOR_GREEN if _fine_held_act == act else image.COLOR_GRAY
                    img.draw_rect(bx, _FBTN_Y, bw, _FBTN_H, color, -1)
                    tx = bx + 9 if len(_labels[i]) <= 2 else bx + 5
                    img.draw_string(tx, _FBTN_Y+10, _labels[i], image.COLOR_WHITE)
                _fine_btns.append((bx, _FBTN_Y, bw, _FBTN_H, act))
                _x0 += bw + _gap

        # Mode 2 自动序列界面 (左下角: Start + 计时)
        if CURRENT_MODE == 2:
            _FBTN_H = 44; _FBTN_Y = H - 52
            _x0 = 5; _gap = 4
            _m2_btns = []
            # Start 按键
            bx = _x0; bw = 60
            sc = image.COLOR_GREEN if _m2_state >= 1 else image.COLOR_GRAY
            img.draw_rect(bx, _FBTN_Y, bw, _FBTN_H, sc, -1)
            img.draw_string(bx+8, _FBTN_Y+10, "Start", image.COLOR_WHITE)
            _m2_btns.append((bx, _FBTN_Y, bw, _FBTN_H, 'start'))
            # 计时器
            bx = _x0 + bw + _gap; bw = 76
            _m2_sec = _m2_timer_ms / 1000.0
            img.draw_rect(bx, _FBTN_Y, bw, _FBTN_H, image.COLOR_WHITE, -1)
            img.draw_string(bx+5, _FBTN_Y+10, f"T:{_m2_sec:.1f}s", image.COLOR_BLACK)

        # 模式按钮栏 (30px高, 在原始图像下方)
        btn_h = 50
        canvas_h = H + dst_h + btn_h
        canvas = image.Image(W, canvas_h, image.Format.FMT_RGB888)
        canvas.draw_image(0, 0, img_warped)
        canvas.draw_image(0, dst_h, img)
        # 画按钮栏背景
        btn_y = H + dst_h
        canvas.draw_rect(0, btn_y, W, btn_h, image.COLOR_BLACK, -1)
        # 三个模式按钮
        btn_w = W // 3
        _mode_btns.clear()
        for i, (label, mid) in enumerate([("M1", 1), ("M2", 2), ("M3", 3)]):
            bx = i * btn_w
            color = image.COLOR_GREEN if CURRENT_MODE == mid else image.COLOR_GRAY
            canvas.draw_rect(bx + 2, btn_y + 2, btn_w - 4, btn_h - 4, color, -1)
            canvas.draw_string(bx + btn_w//2 - 10, btn_y + btn_h//2 - 7, label,
                             image.COLOR_WHITE if CURRENT_MODE == mid else image.COLOR_BLACK)
            _mode_btns.append((bx + 2, btn_y + 2, btn_w - 4, btn_h - 4, mid))

        # ---- Web 推流 (跳帧) ----
        if (WEB_ENABLE or shared.recording) and frame_count % WEB_FRAME_SKIP == 0:
            try:
                jpeg = canvas.to_jpeg(quality=JPEG_QUALITY)
                if not isinstance(jpeg, bytes):
                    jpeg = jpeg.to_bytes()
                if WEB_ENABLE:
                    shared.update(jpeg, {"fps": fps_val, "pos_cm": ball_cm})
                if shared.recording:
                    write_recording_frame(jpeg)
            except Exception as e:
                dbg(f"JPEG err fc={frame_count}: {e}")
                if frame_count <= 5:
                    print("[WEB] JPEG error: " + str(e))
        # 触摸: 模式切换 + Mode 3 目标点设置
        x, y, pressed = ts.read()
        if pressed:
            # 触摸坐标映射回canvas坐标
            x_c, y_c = image.resize_map_pos_reverse(
                W, canvas_h, dis.width(), dis.height(),
                image.Fit.FIT_CONTAIN, x, y)
            # 先检查是否点到模式按钮
            btn_hit = False
            for bx, by, bw, bh, mid in _mode_btns:
                if bx < x_c < bx + bw and by < y_c < by + bh:
                    btn_hit = True
                    if CURRENT_MODE != mid:
                        old_mode = CURRENT_MODE
                        CURRENT_MODE = mid
                        _pid_integral = 0.0   # 切模式清I, 防旧积分污染
                        print(f"[MODE] Switched to Mode {CURRENT_MODE}")
                        if mid == 3:          # 切到M3则重置
                            _target_cm = 0.0; _target_touched = False
                        elif old_mode == 3:   # 切离M3也重置
                            _target_cm = 0.0; _target_touched = False
                        if mid != 2:          # 切离M2则重置序列
                            _m2_state = 0; _m2_timer_ms = 0
                    break
            # Mode 2: Start按键 (防抖)
            if not btn_hit and CURRENT_MODE == 2:
                img_x = x_c; img_y_c = y_c - dst_h
                for bx, by, bw, bh, act in _m2_btns:
                    if act == 'start' and bx < img_x < bx+bw and by < img_y_c < by+bh:
                        if not _m2_start_held:
                            if _m2_state == 0 or _m2_state == 4:
                                _m2_state = 1
                                _m2_timer_start = time.ticks_ms()
                                _m2_timer_ms = 0
                                _pid_integral = 0.0
                                _bias_ramp = 0.0
                                print("[MODE2] Sequence started: 0 -> +5 -> -5")
                            else:
                                _m2_state = 0
                                _m2_timer_ms = 0
                                print("[MODE2] Sequence aborted")
                        _m2_start_held = True
                        btn_hit = True
                        break
                    else:
                        _m2_start_held = False
            else:
                _m2_start_held = False
            # Mode 3: 微调控件 (防抖: 按住只触发一次, 松手才能再触发)
            if not btn_hit and CURRENT_MODE == 3:
                img_x = x_c; img_y_c = y_c - dst_h
                fine_hit = ''
                for bx, by, bw, bh, act in _fine_btns:
                    if bx < img_x < bx+bw and by < img_y_c < by+bh and act != '':
                        fine_hit = act
                        if act != _fine_held_act:   # 不同按键或首次按下才触发
                            if act == '--':   _target_cm -= 1.0
                            elif act == '-':  _target_cm -= 0.1
                            elif act == '+':  _target_cm += 0.1
                            elif act == '++': _target_cm += 1.0
                            elif act == 'set':
                                _target_touched = True
                                _pid_integral = 0.0
                                _bias_ramp = 0.0
                                print(f"[MODE3] Set! Target={_target_cm:+.1f} cm")
                            _target_cm = max(-PIPE_LENGTH/2, min(PIPE_LENGTH/2, _target_cm))
                        btn_hit = True
                        break
                _fine_held_act = fine_hit
            else:
                _fine_held_act = ''   # 没按到微调控件, 清除防抖状态
            # Mode 3: 点击warp图像区域 → 直接设置并激活目标点 (触屏自动Set)
            if not btn_hit and CURRENT_MODE == 3 and y_c < dst_h:
                _eff_w = W - (PIPE_INSET_LEFT + PIPE_INSET_RIGHT)
                _target_cm = (x_c - W / 2.0) * PIPE_LENGTH / max(_eff_w, 1)
                _target_cm = max(-PIPE_LENGTH/2, min(PIPE_LENGTH/2, _target_cm))
                _target_touched = True
                _pid_integral = 0.0
                _bias_ramp = 0.0
                print(f"[MODE3] Target= {_target_cm:+.1f} cm (tap, auto-set)")
        else:
            _fine_held_act = ''   # 松手清零防抖

        dis.show(canvas, fit=image.Fit.FIT_CONTAIN)

    else:
        # 未锁定绿框: 纯检测显示
        if best_det:
            img.draw_rect(best_det.x, best_det.y, best_det.w, best_det.h,
                          color=image.COLOR_RED, thickness=2)
            img.draw_string(best_det.x, max(0, best_det.y - 14),
                            f"sb:{best_det.score:.2f}", color=image.COLOR_RED)

        img.draw_string(4, 4,  f"F:{fps_val:.0f}  no tube", image.COLOR_WHITE)

        # 模式按钮栏 (30px)
        btn_h = 50
        canvas_h = H + btn_h
        canvas = image.Image(W, canvas_h, image.Format.FMT_RGB888)
        canvas.draw_image(0, 0, img)
        btn_y = H
        canvas.draw_rect(0, btn_y, W, btn_h, image.COLOR_BLACK, -1)
        btn_w = W // 3
        _mode_btns.clear()
        for i, (label, mid) in enumerate([("M1", 1), ("M2", 2), ("M3", 3)]):
            bx = i * btn_w
            color = image.COLOR_GREEN if CURRENT_MODE == mid else image.COLOR_GRAY
            canvas.draw_rect(bx + 2, btn_y + 2, btn_w - 4, btn_h - 4, color, -1)
            canvas.draw_string(bx + btn_w//2 - 10, btn_y + btn_h//2 - 7, label,
                             image.COLOR_WHITE if CURRENT_MODE == mid else image.COLOR_BLACK)
            _mode_btns.append((bx + 2, btn_y + 2, btn_w - 4, btn_h - 4, mid))

        # ---- Web 推流 (跳帧) ----
        if (WEB_ENABLE or shared.recording) and frame_count % WEB_FRAME_SKIP == 0:
            try:
                jpeg = canvas.to_jpeg(quality=JPEG_QUALITY)
                if not isinstance(jpeg, bytes):
                    jpeg = jpeg.to_bytes()
                if WEB_ENABLE:
                    shared.update(jpeg, {"fps": fps_val, "pos_cm": 0.0})
                if shared.recording:
                    write_recording_frame(jpeg)
            except Exception as e:
                dbg(f"JPEG2 err fc={frame_count}: {e}")
                if frame_count <= 5:
                    print("[WEB] JPEG error: " + str(e))

        # 触摸切换 (无管道时只切换模式, 不设目标)
        x, y, pressed = ts.read()
        if pressed:
            x_c, y_c = image.resize_map_pos_reverse(
                W, canvas_h, dis.width(), dis.height(),
                image.Fit.FIT_CONTAIN, x, y)
            for bx, by, bw, bh, mid in _mode_btns:
                if bx < x_c < bx + bw and by < y_c < by + bh:
                    if CURRENT_MODE != mid:
                        old_mode = CURRENT_MODE
                        CURRENT_MODE = mid
                        _pid_integral = 0.0   # 切模式清I
                        print(f"[MODE] Switched to Mode {CURRENT_MODE}")
                        if CURRENT_MODE == 3:
                            _target_touched = False
                        if mid != 2:
                            _m2_state = 0; _m2_timer_ms = 0
                    break

        dis.show(canvas, fit=image.Fit.FIT_CONTAIN)

    # 定期GC, 防止内存碎片累积导致崩溃
    if frame_count % 60 == 0:
        gc.collect()
        if frame_count % 300 == 0:
            dbg(f"heartbeat fc={frame_count} fps={fps_val:.0f}")

    # ---- FPS ----
    if frame_count % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_fps_ticks)
        fps_val = 30000.0 / elapsed
        last_fps_ticks = now
        if SERVO_ENABLE and locked_corners is not None:
            duty_now = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * servo_angle / 180.0
            tgt_str = f" tgt:{_target_cm:+.1f}" if (CURRENT_MODE == 3 and _target_touched) else ""
            print(f"[FPS] {fps_val:.1f}  M{CURRENT_MODE} cm:{ball_cm:+.1f}{tgt_str}  v:{_pid_vel:+6.1f}  "
                  f"ang:{servo_angle:.1f}  duty:{duty_now:.3f}%  trk:{'Y' if has_track else 'N'}  "
                  f"P:{p_term:+.1f} D:{d_term:+.1f} I:{i_term:+.1f} B:{_bias:+.1f}")
        else:
            print(f"[FPS] {fps_val:.1f}  cm:{ball_cm:+.1f}")

dbg(f"=== Program exit, total frames={frame_count} ===")
