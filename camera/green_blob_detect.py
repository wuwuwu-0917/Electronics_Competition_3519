# ============================================================
# 绿色色块识别与标定 (MaixPy)
#   - find_blobs 基于 LAB 颜色空间提取绿色区域
#   - mini_corners() 获取四个角点并绘制四边形
#   - 透视变换将倾斜色块矫正到水平方向铺满
#   - 非绿色区域不做任何处理
# ============================================================

from maix import camera, display, image, app, time, gpio, pinmap, err, sys, uart
import cv2, struct, json, threading, socket, os
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

# ---- 绿色阈值 (LAB) --------------------------------------------
GREEN_THRESHOLD = (0, 100, -128, -36, -128, 127)

# ---- 色块检测参数 ----------------------------------------------
MIN_AREA = 150
MERGE    = False
MARGIN   = 10

# ---- 圆形检测参数 ----------------------------------------------
ENABLE_CIRCLE = True   # 是否启用圆形检测
CIRCLE_THR   = 30     # 霍夫圆累加器阈值 (越小越敏感)
CIRCLE_R_MIN = 2      # 最小半径
CIRCLE_R_MAX = 20     # 最大半径
EDGE_HI      = 100    # 内部 Canny 高阈值 (降低以检测弱边缘)

# ---- 区域锁定 ----------------------------------------------------
AREA_SHRINK = 0.6   # 新面积 < 锁定面积*0.6 才更新区域

# ---- 圆形平滑 ----------------------------------------------------
SNAP_DEAD  = 2     # 纯死区: <2px 不动, ≥2px 瞬间跳过去

# ---- 标尺参数 (管子25cm) ----------------------------------------
PIPE_LENGTH = 25.0   # cm

# ---- 调试开关 ----------------------------------------------------
DEBUG = False

# ---- 摄像头 ----------------------------------------------------
CAM_W, CAM_H = 640, 240
cam = camera.Camera(CAM_W, CAM_H, fps=90)
dis = display.Display()

# ---- LED 补光 --------------------------------------------------
LED_ON = True
if LED_ON:
    pin_name = "B25" if sys.device_id() == "maixcam2" else "B3"
    gpio_id  = "GPIOB25" if sys.device_id() == "maixcam2" else "GPIOB3"
    err.check_raise(pinmap.set_pin_function(pin_name, gpio_id), "set pin failed")
    led = gpio.GPIO(gpio_id, gpio.Mode.OUT)
    led.value(1)  # 常亮补光, 增强钢珠反光

# ---- UART 配置 --------------------------------------------------
UART_DEV  = "/dev/ttyS0"
UART_BAUD = 115200
ENABLE_UART = True

# ---- Web 推流配置 ----------------------------------------------
WEB_PORT = 5000
JPEG_QUALITY = 60
WEB_ENABLE = True
RECORD_DIR = "/root/videos"
RECORD_ENABLE = True

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
    """组包: AA 55 | pos_cm(2B signed LE) | CRC16(2B LE) | DD"""
    payload = struct.pack("<h", int(pos_cm * 100))  # 厘米×100 = 0.01cm分辨率
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
        </div>
        <div id="playbackContainer">
            <div id="playbackPlaceholder" class="playback-placeholder">&#9654; 点击下方录像「播放」回放设备端录像</div>
            <img id="streamPlayImg" style="display:none;width:100%;max-width:640px;border-radius:6px;margin:0 auto;" alt="MJPEG回放">
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

// ---- 设备端录像 ----
function toggleDeviceRecord(){
    if(!gDevRecording){
        fetch('/api/record/start')
        .then(function(r){return r.json();})
        .then(function(d){
            if(d.ok){
                gDevRecording=true;
                var btn=document.getElementById('devRecBtn');
                btn.innerHTML='&#9632; 停止设备录像';
                btn.className='btn-rec stop';
                document.getElementById('devRecStatus').textContent='录像中...';
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
            var btn=document.getElementById('devRecBtn');
            btn.innerHTML='&#9679; 开始设备录像';
            btn.className='btn-rec';
            document.getElementById('devRecStatus').textContent=
                '已保存: '+d.frames+'帧, '+d.size_kb+'KB';
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
    var img=document.getElementById('streamPlayImg');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    gIsDevicePlayback=true;
    gDevPlayName=name;
    img.src='/play/'+encodeURIComponent(name);
    img.style.display='block';
    ph.style.display='none';
    btn.style.display='inline-block';
    renderRecordingList();
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
    var img=document.getElementById('streamPlayImg');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    img.src='';
    img.style.display='none';
    ph.style.display='flex';
    btn.style.display='none';
    renderRecordingList();
}

function stopPlayStream(){
    var img=document.getElementById('streamPlayImg');
    if(img.src){
        img.src='';
    }
}

// ---- 格式化 ----
function formatSize(mb){
    return mb>=1?mb.toFixed(1)+' MB':(mb*1024).toFixed(0)+' KB';
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
        h+='<div class="rec-item">'+
            '<span class="rec-name" style="color:#00d4ff;">'+(isPlaying?'&#9654; ':'')+r.name+'</span>'+
            '<span class="rec-meta">'+formatSize(r.size_mb)+'</span>'+
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

# ============================================================
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

# ============================================================
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
                    files.append({"name": f, "size_kb": size // 1024,
                                   "size_mb": size / (1024 * 1024)})
        except Exception as e:
            print(f"[API] list_recordings error: {e}")
        self._json_resp({"recordings": files, "recording": shared.recording})

    def _serve_recording(self, filename):
        """播放录像：读取 .mjpeg 文件并以 MJPEG 流发送"""
        # 安全检查：防止路径穿越
        safe = os.path.basename(filename)
        if not safe.endswith('.mjpeg'):
            print(f"[PLAY] Invalid file type: {safe}")
            self.send_response(403)
            self.end_headers()
            self.wfile.write(b'Invalid file')
            return

        fpath = os.path.join(RECORD_DIR, safe)
        print(f"[PLAY] Serving: {fpath}, exists={os.path.exists(fpath)}, size={os.path.getsize(fpath) if os.path.exists(fpath) else -1}")
        if not os.path.exists(fpath):
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Recording not found')
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        try:
            with open(fpath, 'rb') as f:
                # 跳过头部: MJPEG(5) + version(1) + frame_count(4) = 10 bytes
                f.read(10)
                while True:
                    # 读帧长度 (4 bytes LE)
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
                        time.sleep_ms(33)  # ~30 fps 回放
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


# ============================================================
# 录像功能
# ============================================================

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
        # 文件头: magic MJPEG(5B) + version(1B) + frame_count placeholder(4B) = 10B
        f.write(b'MJPEG\x01\x00\x00\x00\x00')
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
            # 回写帧数到文件头偏移 6 处 (magic 5B + version 1B)
            f.seek(6)
            f.write(struct.pack('<I', shared.record_frame_count))
            f.flush()
            fsize = f.tell()
            f.close()
            info = {"path": shared.record_path,
                     "frames": shared.record_frame_count,
                     "size_kb": fsize // 1024}
            print(f"[REC] Recording stopped: {shared.record_path} "
                  f"frames={shared.record_frame_count} size={fsize}B")
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

# FPS
last_ticks = time.ticks_ms()
frame_cnt = 0
fps = 0.0

print("[INFO] Green blob detection start")

# 锁定区域 (上电识别后锁定, 面积明显缩小时才更新)
locked_corners = None  # (TL, TR, BR, BL)
locked_area = 0
AREA_SHRINK_RATIO = 0.6  # 新面积 < 锁定面积*0.6 才更新

# 上一帧平滑圆
prev_circles = []  # [(cx, cy, r), ...]
circle_miss = 0
COAST_MAX  = 5
JUMP_LIMIT = 30    # 防跳变


def order_corners(corners):
    """按 x 分左右，各组按 y 排上下 → [左上, 右上, 右下, 左下]"""
    pts = sorted(corners, key=lambda p: p[0])
    left  = sorted(pts[:2], key=lambda p: p[1])
    right = sorted(pts[2:], key=lambda p: p[1])
    return [left[0], right[0], right[1], left[1]]


# ---- Web 服务器启动 (守护线程) ------------------------------------
if WEB_ENABLE:
    web_thread = threading.Thread(
        target=start_web_server, args=(WEB_PORT,),
        daemon=True, name="WebServer"
    )
    web_thread.start()
    print(f"[WEB] Web server thread started (port {WEB_PORT})")

# ================================================================
# 主循环
# ================================================================
while not app.need_exit():
    img = cam.read()
    frame_cnt += 1

    if locked_corners is None:
        blobs = img.find_blobs([GREEN_THRESHOLD],
                               area_threshold=MIN_AREA,
                               pixels_threshold=MIN_AREA,
                               merge=MERGE,
                               margin=MARGIN,
                               x_stride=2,
                               y_stride=2)
        if blobs:
            b = max(blobs, key=lambda b: b.area())
            raw_TL, raw_TR, raw_BR, raw_BL = order_corners(b.mini_corners())
            locked_corners = (raw_TL, raw_TR, raw_BR, raw_BL)
            locked_area = b.area()

    if locked_corners is not None:
        ball_pos_cm = 0.0
        TL, TR, BR, BL = locked_corners

        def clamp(p):
            return (max(0, min(CAM_W - 1, int(p[0]))),
                    max(0, min(CAM_H - 1, int(p[1]))))
        TL, TR, BR, BL = clamp(TL), clamp(TR), clamp(BR), clamp(BL)

        src_w = ((TR[0] - TL[0]) + (BR[0] - BL[0])) / 2.0
        src_h = ((BL[1] - TL[1]) + (BR[1] - TR[1])) / 2.0

        scale = (CAM_W - 1) / max(src_w, 1)
        dst_h = int(src_h * scale)
        dst_h = max(dst_h, 2)
        dst_h = dst_h // 2 * 2

        src_flat = [TL[0], TL[1], TR[0], TR[1], BR[0], BR[1], BL[0], BL[1]]
        dst_flat = [0, 0, CAM_W - 1, 0, CAM_W - 1, dst_h - 1, 0, dst_h - 1]

        img_warped = img.perspective(src_flat, dst_flat, CAM_W, dst_h)

        # 三条虚线同色 (黄色)
        cx_0  = CAM_W // 2
        cx_p5 = cx_0 + int(CAM_W * 5.0 / PIPE_LENGTH)
        cx_n5 = cx_0 - int(CAM_W * 5.0 / PIPE_LENGTH)
        for cy in range(0, dst_h, 10):
            ye = min(cy + 5, dst_h)
            img_warped.draw_line(cx_0, cy, cx_0, ye, image.COLOR_YELLOW, 1)
            img_warped.draw_line(cx_p5, cy, cx_p5, ye, image.COLOR_YELLOW, 1)
            img_warped.draw_line(cx_n5, cy, cx_n5, ye, image.COLOR_YELLOW, 1)

        # 水平中心实线 (红色, 与球同色)
        mid_y = dst_h // 2
        img_warped.draw_line(0, mid_y, CAM_W - 1, mid_y, image.COLOR_RED, 1)

        # 圆形检测 + 死区 (单球)
        if ENABLE_CIRCLE:
            np_img = image.image2cv(img_warped, ensure_bgr=False, copy=False)
            np_gray = cv2.cvtColor(np_img, cv2.COLOR_BGR2GRAY)
            np_gray = cv2.GaussianBlur(np_gray, (5, 5), 0)

            raw = cv2.HoughCircles(np_gray, cv2.HOUGH_GRADIENT,
                                   dp=1.5, minDist=10,
                                   param1=EDGE_HI, param2=CIRCLE_THR,
                                   minRadius=CIRCLE_R_MIN,
                                   maxRadius=CIRCLE_R_MAX)
            best = None
            if raw is not None and len(raw) > 0:
                circles = raw[0]
                best = max(circles, key=lambda c: c[2])
                rx, ry, rr = best[0], best[1], best[2]

            if best is not None:
                if prev_circles:
                    pc = prev_circles[0]
                    d2 = (rx - pc[0])**2 + (ry - pc[1])**2
                    if d2 >= JUMP_LIMIT * JUMP_LIMIT:
                        best = None
                if best is not None:
                    circle_miss = 0
                    if prev_circles:
                        pc = prev_circles[0]
                        d2 = (rx - pc[0])**2 + (ry - pc[1])**2
                        if d2 < SNAP_DEAD * SNAP_DEAD:
                            pass
                        else:
                            prev_circles = [(rx, ry, rr)]
                    else:
                        prev_circles = [(rx, ry, rr)]
            else:
                circle_miss += 1
                if circle_miss > COAST_MAX:
                    prev_circles = []

            ball_pos_cm = 0.0
            for sc in prev_circles:
                ball_x = int(sc[0])
                img_warped.draw_circle(ball_x, mid_y, 6, image.COLOR_RED, -1)

                ball_pos_cm = (ball_x - CAM_W / 2) * PIPE_LENGTH / CAM_W
                if ser is not None:
                    ser.write(build_packet(ball_pos_cm))
                print("Ball: %.2f cm  fps:%.1f" % (ball_pos_cm, fps))

        # 画布: 矫正图在上, 原图在下
        canvas_h = CAM_H + dst_h
        canvas = image.Image(CAM_W, canvas_h, image.Format.FMT_RGB888)
        canvas.draw_image(0, 0, img_warped)
        canvas.draw_image(0, dst_h, img)
        # Web 推流
        if WEB_ENABLE or RECORD_ENABLE:
            try:
                jpeg = canvas.to_jpeg(quality=JPEG_QUALITY)
                if not isinstance(jpeg, bytes):
                    jpeg = jpeg.to_bytes()
                if WEB_ENABLE:
                    shared.update(jpeg, {"fps": fps, "pos_cm": ball_pos_cm})
                if RECORD_ENABLE:
                    write_recording_frame(jpeg)
            except Exception as e:
                if frame_cnt <= 5:
                    print(f"[WEB] JPEG encode failed: {e}")

        dis.show(canvas)

    else:
        prev_circles = []
        circle_miss = 0
        dis.show(img)

    if frame_cnt % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_ticks)
        fps = 30000.0 / elapsed
        last_ticks = now
