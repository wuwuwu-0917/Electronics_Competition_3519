
# ============================================================
# YOLO 钢球检测 + 绿色管道透视矫正 (MaixPy)
#   - 上电识别绿色管道 → 透视变换铺平 (仅一次)
#   - LED 补光 + UART 发送 (CRC16 校验包)
#   - YOLO 在原图上推理, warp 图用于参考线和位置计算
# ============================================================

from maix import camera, display, image, nn, app, comm, time, uart, gpio, pinmap, pwm, err, sys, touchscreen
from maix.ext_dev import imu
import struct, os, json, threading, math
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
RECORD_ENABLE = True

# ---- 标尺参数 (管子长度 cm) -------------------------------------
PIPE_LENGTH = 25.0   # cm

# ---- 管道端点内缩 (映射后 ±12.5cm 端点各往里缩N像素) --------------
PIPE_END_INSET = 10   # px, 全局可调

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
SERVO_FREQ    = 50        # 标准舵机 50Hz
SERVO_MIN_DUTY = 5.0      # 0° 对应占空比 % (标准舵机 1ms)
SERVO_MAX_DUTY = 10.0     # 180° 对应占空比 % (标准舵机 2ms)

# ---- 双模式 PD —— 稳态精细, 扰动全力 ----
BAL_CENTER    = 90.0
BAL_MAX_ANGLE = 150.0
BAL_MIN_ANGLE = 30.0
# 稳态模式
S_KP = 8.0;   S_KD = 4.0;   S_DB = 0.2;  S_RATE = 300.0
# 扰动模式
D_KP = 10.0;  D_KD = 7.0;   D_DB = 0.0;  D_RATE = 600.0
# 模式切换阈值
SW_ERR = 2.5    # cm, |error|超过此值→扰动
SW_VEL = 10.0   # cm/s, |速度|超过此值→扰动
SW_EXIT_ERR = 1.5  # cm, 退出扰动需|error|<此值
SW_EXIT_VEL = 5.0  # cm/s, 退出扰动需|速度|<此值
# 测试模式 (触摸屏切换)
CURRENT_MODE = 1     # 1=双模PD, 2=舵机摆动, 3=不动作
# 内部状态
_pid_integral   = 0.0
_pid_last_out   = 90.0
_pid_last_ms    = 0
_pid_disturbance = False

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
    print(f"[INFO] Dual-Mode PD: "
          f"Steady(KP={S_KP} KD={S_KD} DB={S_DB} Rate={S_RATE})  "
          f"Disturb(KP={D_KP} KD={D_KD} DB={D_DB} Rate={D_RATE})  "
          f"Switch(err>{SW_ERR} vel>{SW_VEL})")
    print("[INFO] Servo ready.")

# ---- 触摸屏 ------------------------------------------------------
ts = touchscreen.TouchScreen()
_mode_btns = []   # 触摸按钮区域 [(x,y,w,h, label, mode_id), ...]

# ---- 运动补偿 (IMU前馈) ------------------------------------------
_imu = imu.IMU("default", mode=imu.Mode.DUAL,
               acc_scale=imu.AccScale.ACC_SCALE_4G,
               acc_odr=imu.AccOdr.ACC_ODR_1000,
               gyro_scale=imu.GyroScale.GYRO_SCALE_500DPS,
               gyro_odr=imu.GyroOdr.GYRO_ODR_8000)
print("[IMU] Checking calibration...")
if _imu.calib_gyro_exists():
    _imu.load_calib_gyro()
    print("[IMU] Loaded previous calibration")
else:
    print("[IMU] Calibrating — keep device STILL for 10s...")
    _imu.calib_gyro(10000)
    print("[IMU] Calibration saved.")
K_ACC_COMP   = 1.0    # 加速度前馈增益 (deg per m/s² 等效, 可调)
_imu_acc_filt = 0.0    # 加速度低通滤波状态

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
        """MJPEG 流：持续发送 multipart JPEG 帧"""
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        self.end_headers()

        last_sent = None  # 避免重复发送相同帧
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
            else:
                time.sleep_ms(10)

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
                # 跳过头部: MJPEG(5) + version(1) + frame_count(4) + elapsed_ms(4) = 14 bytes
                f.read(14)
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
                        time.sleep_ms(33)
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
    return '192.168.43.130'


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
            <img id="streamLive" src="/video_feed" alt="实时视频流">
        </div>
    </div>

    <!-- ====== 设备端录像 ====== -->
    <div class="section">
        <div class="section-header"><span class="icon">&#128190;</span> 设备端录像 (本地储存)</div>
        <div class="rec-row">
            <button id="devRecBtn" class="btn-rec" onclick="toggleDeviceRecord()">&#9679; 开始设备录像</button>
            <span id="devRecStatus" style="color:#888;font-size:13px;"></span>
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
            <div id="playbackPlaceholder" class="playback-placeholder">&#9654; 点击下方录像「播放」回放设备端录像</div>
            <img id="streamPlayImg" style="display:none;width:100%;max-width:640px;border-radius:6px;margin:0 auto;" alt="MJPEG回放">
        </div>
        <div id="playbackBar" style="display:none;margin-top:8px;">
            <input type="range" id="seekBar" min="0" max="100" value="0" step="1"
                   style="width:100%;max-width:640px;display:block;margin:0 auto;accent-color:#e74c3c;"
                   oninput="onSeekPreview(this.value)" onchange="onSeekDo(this.value)" onmousedown="gIsSeeking=true" onmouseup="gIsSeeking=false">
            <div style="display:flex;justify-content:space-between;max-width:640px;margin:2px auto 0;font-size:11px;color:#666;">
                <span id="seekLabelStart">00:00</span><span id="seekLabelEnd">00:00</span>
            </div>
        </div>
    </div>

    <!-- ====== 录像列表 ====== -->
    <div class="section">
        <div class="section-header">
            <span class="icon">&#128193;</span> 录像列表
            <button class="btn-sm" onclick="renderRecordingList()">&#8635; 刷新</button>
        </div>
        <div id="recList" class="rec-list">
            <div class="empty-hint">暂无录像，点击「开始录像」进行录制</div>
        </div>
    </div>
</div>

<script>//<![CDATA[
var gDevRecording=false,gDeviceRecordings=[],gIsDevicePlayback=false,gDevPlayName='';
var gRecTimer=null,gRecStartTime=0;
var gPlayTotalFrames=0,gPlayShowFps=30,gPlayPosTimer=null,gPlayStartTime=0,gPlayStartFrame=0;
var PLAYBACK_FPS=30;
var gIsSeeking=false;

function fmtDuration(sec){
    var m=Math.floor(sec/60);
    var s=Math.floor(sec%60);
    return (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}

// ---- 设备端录像 ----
function toggleDeviceRecord(){
    if(!gDevRecording){
        fetch('/api/record/start')
        .then(function(r){return r.json();})
        .then(function(d){
            if(d.ok){
                gDevRecording=true;
                gRecStartTime=Date.now();
                var btn=document.getElementById('devRecBtn');
                btn.innerHTML='&#9632; 停止设备录像';
                btn.className='btn-rec stop';
                gRecTimer=setInterval(function(){
                    var el=(Date.now()-gRecStartTime)/1000;
                    document.getElementById('devRecStatus').innerHTML='&#9202; '+fmtDuration(el);
                },500);
            }else{
                alert('无法开始设备录像');
            }
        })
        .catch(function(e){console.error(e);});
    }else{
        fetch('/api/record/stop')
        .then(function(r){return r.json();})
        .then(function(d){
            gDevRecording=false;
            if(gRecTimer){clearInterval(gRecTimer);gRecTimer=null;}
            var btn=document.getElementById('devRecBtn');
            btn.innerHTML='&#9679; 开始设备录像';
            btn.className='btn-rec';
            var el=(Date.now()-gRecStartTime)/1000;
            document.getElementById('devRecStatus').innerHTML='已保存 '+fmtDuration(el);
            loadDeviceRecordings();
        })
        .catch(function(e){console.error(e);});
    }
}

function loadDeviceRecordings(){
    fetch('/api/recordings')
    .then(function(r){return r.json();})
    .then(function(d){
        gDeviceRecordings=d.recordings||[];
        renderRecordingList();
    })
    .catch(function(e){console.error(e);});
}

function playDeviceRecord(name){
    stopPlayStream();
    var rec=gDeviceRecordings.find(function(r){return r.name===name;});
    gPlayTotalFrames=(rec&&rec.frames)?rec.frames:0;
    gPlayStartFrame=0;

    var img=document.getElementById('streamPlayImg');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    var bar=document.getElementById('playbackBar');
    var tm=document.getElementById('playbackTime');
    var seek=document.getElementById('seekBar');

    // 时间显示用真实录制时长, 进度走帧用播放帧率(30fps=33ms/帧)
    var totalSec = (rec&&rec.elapsed_ms) ? rec.elapsed_ms/1000 : (gPlayTotalFrames/30);
    gPlayShowFps = (gPlayTotalFrames>0 && totalSec>0) ? gPlayTotalFrames/totalSec : 30;

    gIsDevicePlayback=true;
    gDevPlayName=name;
    gPlayStartTime=Date.now();
    img.src='/play/'+encodeURIComponent(name)+'?t='+Date.now();
    img.style.display='block';
    ph.style.display='none';
    btn.style.display='inline-block';
    tm.style.display='inline';
    if(gPlayTotalFrames>0){
        bar.style.display='block';
        seek.max=gPlayTotalFrames;
        seek.value=0;
        document.getElementById('seekLabelStart').textContent='00:00';
        document.getElementById('seekLabelEnd').textContent=fmtDuration(totalSec);
    }

    if(gPlayPosTimer)clearInterval(gPlayPosTimer);
    gPlayPosTimer=setInterval(function(){
        if(!gIsSeeking){
            var el=(Date.now()-gPlayStartTime)/1000;
            var curFrame=Math.floor(gPlayStartFrame+el*PLAYBACK_FPS);
            if(curFrame<=gPlayTotalFrames){
                seek.value=curFrame;
                // 时间用录制帧率换算(显示真实时刻)
                tm.textContent=fmtDuration(curFrame/gPlayShowFps);
            }else{
                seek.value=gPlayTotalFrames;
                tm.textContent=fmtDuration(gPlayTotalFrames/gPlayShowFps);
            }
        }
    },300);

    renderRecordingList();
}

function onSeekPreview(val){
    var f=parseInt(val);
    document.getElementById('playbackTime').textContent=fmtDuration(f/gPlayShowFps);
    document.getElementById('seekLabelStart').textContent=fmtDuration(f/gPlayShowFps);
}
function onSeekDo(val){
    var f=parseInt(val);
    gPlayStartFrame=f;
    gPlayStartTime=Date.now();
    var img=document.getElementById('streamPlayImg');
    img.src='/play/'+encodeURIComponent(gDevPlayName)+'?start='+f+'&t='+Date.now();
    document.getElementById('playbackTime').textContent=fmtDuration(f/gPlayShowFps);
    document.getElementById('seekLabelStart').textContent=fmtDuration(f/gPlayShowFps);
    setTimeout(function(){gIsSeeking=false;},500);
}

function downloadDeviceRecord(name){
    var a=document.createElement('a');
    a.href='/play/'+encodeURIComponent(name);
    a.download=name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
}

function deleteDeviceRecord(name){
    if(!confirm('删除设备录像 '+name+' ?'))return;
    fetch('/api/recordings/'+encodeURIComponent(name)+'/delete')
    .then(function(r){return r.json();})
    .then(function(d){
        if(d.ok){
            if(gIsDevicePlayback&&gDevPlayName===name)stopPlayback();
            loadDeviceRecordings();
        }
    })
    .catch(function(e){console.error(e);});
}

// 设备录像索引包装 (避免 onclick 属性中转义文件名)
function playDeviceRecordByIdx(idx){
    if(idx>=0&&idx<gDeviceRecordings.length){
        playDeviceRecord(gDeviceRecordings[idx].name);
    }
}
function downloadDeviceRecordByIdx(idx){
    if(idx>=0&&idx<gDeviceRecordings.length){
        downloadDeviceRecord(gDeviceRecordings[idx].name);
    }
}
function deleteDeviceRecordByIdx(idx){
    if(idx>=0&&idx<gDeviceRecordings.length){
        deleteDeviceRecord(gDeviceRecordings[idx].name);
    }
}

// ---- 回放 ----
function stopPlayback(){
    stopPlayStream();
    gIsDevicePlayback=false;
    gDevPlayName='';
    gPlayTotalFrames=0;
    gPlayStartFrame=0;
    if(gPlayPosTimer){clearInterval(gPlayPosTimer);gPlayPosTimer=null;}
    var img=document.getElementById('streamPlayImg');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    var bar=document.getElementById('playbackBar');
    var tm=document.getElementById('playbackTime');
    img.src='';
    img.style.display='none';
    ph.style.display='flex';
    btn.style.display='none';
    bar.style.display='none';
    tm.style.display='none';
    renderRecordingList();
}

function stopPlayStream(){
    var img=document.getElementById('streamPlayImg');
    if(img.src){
        img.src='';
    }
}

// ---- 渲染录像列表 ----
function renderRecordingList(){
    var list=document.getElementById('recList');
    var devRecs=gDeviceRecordings||[];
    if(devRecs.length===0){
        list.innerHTML='<div class="empty-hint">暂无录像，点击「开始设备录像」进行录制</div>';
        return;
    }
    var h='';
    for(var i=0;i<devRecs.length;i++){
        var r=devRecs[i];
        var isPlaying=(gIsDevicePlayback&&gDevPlayName===r.name);
        var dur=(r.elapsed_ms?fmtDuration(r.elapsed_ms/1000):(r.frames?fmtDuration(r.frames/30):''));
        h+='<div class="rec-item">'+
            '<span class="rec-name" style="color:#00d4ff;">'+(isPlaying?'&#9654; ':'')+r.name+'</span>'+
            '<span class="rec-meta">'+dur+'</span>'+
            '<button class="btn-play" onclick="playDeviceRecordByIdx('+i+')">播放</button>'+
            '<button class="btn-dl" onclick="downloadDeviceRecordByIdx('+i+')">下载</button>'+
            '<button class="btn-del" onclick="deleteDeviceRecordByIdx('+i+')">删除</button>'+
            '</div>';
    }
    list.innerHTML=h;
}

// ---- 启动 ----
loadDeviceRecordings();
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

# 单球追踪器
tracker = BallTracker()

# ================================================================
# 主循环
# ================================================================
while not app.need_exit():
    img = cam.read()
    frame_count += 1
    # 消除 Pyright possibly-unbound 误报 (运行时始终会被覆盖)
    servo_angle = BAL_CENTER; p_term = d_term = i_term = 0.0
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
        # 管道端点虚线 (青色, ±12.5cm 各内缩 PIPE_END_INSET)
        cx_left_end  = PIPE_END_INSET
        cx_right_end = W - 1 - PIPE_END_INSET
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
            effective_w = W - 2 * PIPE_END_INSET
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
        p_term = d_term = i_term = 0.0; _pid_vel = _feedforward = 0.0
        if CURRENT_MODE == 3:
            servo_angle = BAL_CENTER
            duty = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * servo_angle / 180.0
            servo.duty(duty)
        elif CURRENT_MODE == 2:
            now_ms = time.ticks_ms()
            phase = (now_ms % 2000) / 2000.0
            if phase < 0.5:
                servo_angle = 60.0 + 60.0 * (phase / 0.5)
            else:
                servo_angle = 120.0 - 60.0 * ((phase - 0.5) / 0.5)
            duty = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * servo_angle / 180.0
            servo.duty(duty)
        elif has_track:
            now_ms = time.ticks_ms()
            if _pid_last_ms == 0:
                dt = 0.016
            else:
                dt = (now_ms - _pid_last_ms) / 1000.0
                if dt <= 0 or dt > 1.0:
                    dt = 0.016
            _pid_last_ms = now_ms

            error = ball_cm
            abs_err = abs(error)

            # Kalman速度 (零滞后)
            if dt > 0.001:
                _pid_vel = tracker.vel_x() * (PIPE_LENGTH / max(W - 2 * PIPE_END_INSET, 1)) / dt

            # === 模式切换 (宽滞后: 快进慢出, 防D模式滞留近中心) ===
            if abs_err > SW_ERR or abs(_pid_vel) > SW_VEL:
                _pid_disturbance = True
            elif abs_err < SW_EXIT_ERR and abs(_pid_vel) < SW_EXIT_VEL:
                _pid_disturbance = False

            if _pid_disturbance:
                # === 扰动模式: 大力但有上限, 防bang-bang ===
                _kp = D_KP; _kd = D_KD; _db = D_DB; _rate = D_RATE
                _pid_integral = 0.0
                if abs_err < _db:
                    p_term = 0.0
                else:
                    s = 1.0 if error > 0 else -1.0
                    p_term = _kp * (error - s * _db)
                d_term = _kd * _pid_vel
                if d_term > 60.0: d_term = 60.0
                elif d_term < -60.0: d_term = -60.0
                i_term = 0.0
            else:
                # === 稳态模式: 精细控制 ===
                _kp = S_KP; _kd = S_KD; _db = S_DB; _rate = S_RATE
                if abs_err < _db:
                    p_term = 0.0
                else:
                    s = 1.0 if error > 0 else -1.0
                    p_term = _kp * (error - s * _db)
                d_term = _kd * _pid_vel
                if d_term > 40.0: d_term = 40.0
                elif d_term < -40.0: d_term = -40.0
                # 慢I: 纠正稳态偏移 (放宽激活条件, 增大上限)
                if abs_err < 1.2 and abs(_pid_vel) < 3.0:
                    _pid_integral += error * dt * 1.2
                    _pid_integral = max(-6.0, min(6.0, _pid_integral))
                else:
                    _pid_integral *= 0.85
                i_term = _pid_integral

            # === IMU 运动补偿前馈 ===
            try:
                imu_data = _imu.read_all(calib_gryo=True, radian=False)
                _imu_acc_filt = 0.5 * imu_data.acc.x + 0.5 * _imu_acc_filt
                if abs(_imu_acc_filt) > 0.3:
                    _feedforward = K_ACC_COMP * math.atan2(_imu_acc_filt, 9.8) * 57.3
                else:
                    _feedforward = 0.0
            except Exception:
                _feedforward = 0.0

            # === 输出合成 ===
            raw_out = BAL_CENTER + p_term + d_term + i_term + _feedforward
            max_step = _rate * dt
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
        # 球心圆点
        if has_track:
            bw_x, _ = _point_to_warp(tcx, tcy, TL, TR, BR, BL, W, dst_h)
            img_warped.draw_circle(int(bw_x + BALL_OFFSET_X), mid_y, 6, image.COLOR_RED, -1)

        # 在原图上画追踪框
        if has_track:
            bx = int(tcx - tw // 2)
            by = int(tcy - th // 2)
            img.draw_rect(bx, by, int(tw), int(th),
                          color=image.COLOR_GREEN, thickness=2)
            img.draw_string(bx, max(0, by - 14),
                            f"sb:{tscore:.2f}", color=image.COLOR_GREEN)

        img.draw_string(4, 4,  f"F:{fps_val:.0f} cm:{ball_cm:+.1f} ang:{servo_angle:.0f} {'T' if has_track else '!'}", image.COLOR_WHITE)

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
        if (WEB_ENABLE or RECORD_ENABLE) and frame_count % WEB_FRAME_SKIP == 0:
            try:
                jpeg = canvas.to_jpeg(quality=JPEG_QUALITY)
                if not isinstance(jpeg, bytes):
                    jpeg = jpeg.to_bytes()
                if WEB_ENABLE:
                    shared.update(jpeg, {"fps": fps_val, "pos_cm": ball_cm})
                if RECORD_ENABLE:
                    write_recording_frame(jpeg)
            except Exception as e:
                if frame_count <= 5:
                    print("[WEB] JPEG error: " + str(e))

        # 触摸切换模式
        x, y, pressed = ts.read()
        if pressed:
            # 触摸坐标映射回canvas坐标
            x_c, y_c = image.resize_map_pos_reverse(
                W, canvas_h, dis.width(), dis.height(),
                image.Fit.FIT_CONTAIN, x, y)
            for bx, by, bw, bh, mid in _mode_btns:
                if bx < x_c < bx + bw and by < y_c < by + bh:
                    if CURRENT_MODE != mid:
                        CURRENT_MODE = mid
                        print(f"[MODE] Switched to Mode {CURRENT_MODE}")
                    break

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
        if (WEB_ENABLE or RECORD_ENABLE) and frame_count % WEB_FRAME_SKIP == 0:
            try:
                jpeg = canvas.to_jpeg(quality=JPEG_QUALITY)
                if not isinstance(jpeg, bytes):
                    jpeg = jpeg.to_bytes()
                if WEB_ENABLE:
                    shared.update(jpeg, {"fps": fps_val, "pos_cm": 0.0})
                if RECORD_ENABLE:
                    write_recording_frame(jpeg)
            except Exception as e:
                if frame_count <= 5:
                    print("[WEB] JPEG error: " + str(e))

        # 触摸切换
        x, y, pressed = ts.read()
        if pressed:
            x_c, y_c = image.resize_map_pos_reverse(
                W, canvas_h, dis.width(), dis.height(),
                image.Fit.FIT_CONTAIN, x, y)
            for bx, by, bw, bh, mid in _mode_btns:
                if bx < x_c < bx + bw and by < y_c < by + bh:
                    if CURRENT_MODE != mid:
                        CURRENT_MODE = mid
                        print(f"[MODE] Switched to Mode {CURRENT_MODE}")
                    break

        dis.show(canvas, fit=image.Fit.FIT_CONTAIN)

    # ---- FPS ----
    if frame_count % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_fps_ticks)
        fps_val = 30000.0 / elapsed
        last_fps_ticks = now
        if SERVO_ENABLE and locked_corners is not None:
            duty_now = SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * servo_angle / 180.0
            mode = 'D' if _pid_disturbance else 'S'
            print(f"[FPS] {fps_val:.1f}  M{CURRENT_MODE} cm:{ball_cm:+.1f}  v:{_pid_vel:+6.1f}  "
                  f"ang:{servo_angle:.1f}  duty:{duty_now:.3f}%  trk:{'Y' if has_track else 'N'}  "
                  f"{mode} P:{p_term:+.1f} D:{d_term:+.1f} I:{i_term:+.1f} FF:{_feedforward:+.1f}")
        else:
            print(f"[FPS] {fps_val:.1f}  cm:{ball_cm:+.1f}")
