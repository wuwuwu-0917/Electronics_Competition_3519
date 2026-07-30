# ============================================================
# 钢球检测 Web 推流程序 (MaixCAM2 原生版)
#   - MaixPy NPU 推理 + 追踪 + UART（完全复用 ball_to_center.py）
#   - 新增: WiFi 网页 MJPEG 推流（内置 http.server，零 pip 依赖）
#   - 绿框标出钢球，红线连接画面中心
#   - 三条竖直虚线 + 距离标注 + ball_zone 巡线引导
# ============================================================

from maix import camera, display, image, nn, app, time, uart, gpio, pinmap, sys, err
import math, os, struct, json, threading, socket
from http.server import HTTPServer, BaseHTTPRequestHandler

# 尝试获取多线程 HTTP 服务器（避免 MJPEG 流阻塞其他请求）
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
        # MaixPy 环境下 socketserver 可能不存在，手动实现多线程
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

# ---- 检测参数 -------------------------------------------------
CONF_TH = 0.60
IOU_TH  = 0.50
TARGET_LABELS = ["sb"]

# ---- 追踪器参数 -----------------------------------------------
CONFIRM_HITS = 1
COAST_MAX    = 3
MATCH_DIST   = 80
MERGE_DIST   = 15

# ---- 颜色常量 -------------------------------------------------
COLOR_GREEN   = image.COLOR_GREEN
COLOR_RED     = image.COLOR_RED
COLOR_YELLOW  = image.COLOR_YELLOW
COLOR_WHITE   = image.COLOR_WHITE
COLOR_CROSS   = image.Color(0, 255, 255)

# ---- 绘制开关 -------------------------------------------------
SHOW_ONLY_NEAREST = 1

# ---- 巡线引导 ------------------------------------------------
CENTER_THRESH = 20

# ---- UART 配置 ------------------------------------------------
UART_DEV  = "/dev/ttyS0"
UART_BAUD = 115200
ENABLE_UART = True

# ---- Web 推流配置 ----------------------------------------------
WEB_PORT = 5000               # HTTP 端口
JPEG_QUALITY = 60              # JPEG 质量 50-85
WEB_ENABLE = True              # 是否启用 Web 推流

# ---- 录像配置 --------------------------------------------------
RECORD_DIR = "/root/videos"    # 录像保存目录
RECORD_ENABLE = False          # 服务端录像已禁用（改用浏览器 MediaRecorder）

# ============================================================
# CRC16 + UART 数据包
# ============================================================

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
    payload = struct.pack("<BbHH",
                          has_ball & 1,
                          max(-128, min(127, ball_zone)),
                          int(cx), int(cy))
    crc = crc16(payload)
    return b'\xAA\x55' + payload + struct.pack("<H", crc) + b'\xDD'


# ============================================================
# 追踪器 (与 ball_to_center.py 完全一致)
# ============================================================

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
            px, py = t.predict()
            best_di = -1
            best_dd = d2max + 1
            for di, d in enumerate(detections):
                if det_to_track[di] >= 0:
                    continue
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

            t.vx = 0.7 * (d[0] - t.cx) + 0.3 * t.vx
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

# ============================================================
# 绘制工具
# ============================================================

def draw_dashed_line(img, x0, y0, x1, y1, color, dash_len=6, gap_len=4):
    """画虚线：水平或垂直方向"""
    if y0 == y1:
        x = x0
        end = x1
        while x < end:
            xe = min(x + dash_len, end)
            img.draw_line(x, y0, xe, y0, color, 1)
            x = xe + gap_len
    elif x0 == x1:
        y = y0
        end = y1
        while y < end:
            ye = min(y + dash_len, end)
            img.draw_line(x0, y, x0, ye, color, 1)
            y = ye + gap_len


# ============================================================
# 巡线引导
# ============================================================

def calc_ball_zone(t, cx, w):
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


# ============================================================
# 共享状态（主线程写入，HTTP 线程读取）
# ============================================================

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg_bytes = b''
        self.status = {
            "fps": 0.0, "ball_zone": 0, "has_ball": 0,
            "track_count": 0, "frame_no": 0,
            "tx_bytes": 0, "rx_bytes": 0,
        }
        # 录像状态
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


# ============================================================
# HTML 页面
# ============================================================

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
.rec-timer{font-family:'Cascadia Code',Consolas,monospace;color:#e74c3c;font-size:17px;font-weight:bold}
.rec-size{color:#888;font-size:13px}
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
.unsupported{color:#e74c3c;font-size:12px;padding:8px;background:#2a0a0a;border-radius:4px;margin-top:8px;display:none}
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

    <!-- ====== 浏览器录像 ====== -->
    <div class="section">
        <div class="section-header"><span class="icon">&#127916;</span> 浏览器录像</div>
        <div class="rec-row">
            <button id="recBtn" class="btn-rec" onclick="toggleBrowserRecord()">&#9679; 开始录像</button>
            <span id="recTimer" class="rec-timer" style="display:none;">00:00</span>
            <span id="recSize" class="rec-size"></span>
        </div>
        <div id="unsupportedMsg" class="unsupported">&#9888; 你的浏览器不支持 MediaRecorder API，请使用 Chrome 或 Edge</div>
    </div>

    <!-- ====== 录像回放 ====== -->
    <div class="section">
        <div class="section-header">
            <span class="icon">&#127910;</span> 录像回放
            <button id="btnStopPlay" class="btn-stop-play" onclick="stopPlayback()" style="display:none;">&#9632; 停止回放</button>
        </div>
        <div id="playbackContainer">
            <div id="playbackPlaceholder" class="playback-placeholder">&#9654; 在下方录像列表点击「播放」查看录像</div>
            <video id="streamPlay" style="display:none;width:100%;max-width:640px;border-radius:6px;margin:0 auto;" controls playsinline></video>
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

<!-- 隐藏画布：用于从 MJPEG <img> 抓帧给 MediaRecorder -->
<canvas id="captureCanvas" style="display:none;"></canvas>

<script>//<![CDATA[
var gRecording=false,gRecTimer=null;
var gCanvas=document.getElementById('captureCanvas');
var gCtx=gCanvas.getContext('2d');
var gCaptureTimer=null;
var gMediaRecorder=null;
var gRecordedChunks=[];
var gRecordings=[];
var gRecordStartTime=0;
var gRecordElapsedTimer=null;
var gCurrentPlayIdx=-1;
var gSupported=true;

// ---- 检测浏览器是否支持 MediaRecorder ----
if(typeof MediaRecorder==='undefined' || !gCanvas.captureStream){
    gSupported=false;
    document.getElementById('unsupportedMsg').style.display='block';
    document.getElementById('recBtn').disabled=true;
    document.getElementById('recBtn').style.opacity='0.5';
}

// ---- 画布帧捕获：通过 HTTP 拉取独立 JPEG 帧，可靠触发 canvas 更新 ----
var gSnapImg=new Image();
var gSnapPending=false;
function fetchSnapshot(){
    if(gSnapPending)return;
    gSnapPending=true;
    gSnapImg.src='/snapshot?t='+Date.now();
}
gSnapImg.onload=function(){
    if(gCanvas.width!==gSnapImg.naturalWidth){
        gCanvas.width=gSnapImg.naturalWidth;
        gCanvas.height=gSnapImg.naturalHeight;
    }
    gCtx.drawImage(gSnapImg,0,0);
    gSnapPending=false;
};
gSnapImg.onerror=function(){gSnapPending=false;};
function startFrameCapture(){
    gCaptureTimer=setInterval(fetchSnapshot,16);
}

// ---- 浏览器录像 ----
function toggleBrowserRecord(){
    if(!gSupported)return;
    if(!gRecording){
        startBrowserRecord();
    }else{
        stopBrowserRecord();
    }
}

function startBrowserRecord(){
    gRecordedChunks=[];
    gRecordStartTime=Date.now();
    gRecording=true;

    var btn=document.getElementById('recBtn');
    btn.innerHTML='&#9632; 停止录像';
    btn.className='btn-rec stop';

    // 显示计时器
    var timer=document.getElementById('recTimer');
    timer.style.display='inline';
    timer.textContent='00:00';
    gRecordElapsedTimer=setInterval(updateRecTimer,500);
    document.getElementById('recSize').textContent='';

    // 创建 MediaRecorder
    var stream=gCanvas.captureStream(60);
    var mime='video/webm;codecs=vp9';
    if(!MediaRecorder.isTypeSupported(mime))mime='video/webm;codecs=vp8';
    if(!MediaRecorder.isTypeSupported(mime))mime='video/webm';

    gMediaRecorder=new MediaRecorder(stream,{mimeType:mime});
    gMediaRecorder.ondataavailable=function(e){
        if(e.data&&e.data.size>0)gRecordedChunks.push(e.data);
        updateRecSize();
    };
    gMediaRecorder.onstop=onRecordStop;
    gMediaRecorder.onerror=function(e){
        console.error('MediaRecorder error:',e);
        stopBrowserRecord();
    };
    gMediaRecorder.start(1000);
}

function updateRecTimer(){
    var elapsed=Math.floor((Date.now()-gRecordStartTime)/1000);
    var m=Math.floor(elapsed/60);
    var s=elapsed%60;
    document.getElementById('recTimer').textContent=
        (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}

function updateRecSize(){
    var total=0;
    for(var i=0;i<gRecordedChunks.length;i++)total+=gRecordedChunks[i].size;
    document.getElementById('recSize').textContent=
        total>=1048576?(total/1048576).toFixed(1)+' MB':(total/1024).toFixed(0)+' KB';
}

function stopBrowserRecord(){
    if(gMediaRecorder&&gMediaRecorder.state!=='inactive'){
        gMediaRecorder.requestData();
        gMediaRecorder.stop();
    }
    if(gRecordElapsedTimer){clearInterval(gRecordElapsedTimer);gRecordElapsedTimer=null;}
    gRecording=false;

    var btn=document.getElementById('recBtn');
    btn.innerHTML='&#9679; 开始录像';
    btn.className='btn-rec';
    document.getElementById('recTimer').style.display='none';
}

function onRecordStop(){
    var mime=gMediaRecorder?gMediaRecorder.mimeType:'video/webm';
    var blob=new Blob(gRecordedChunks,{type:mime});
    var url=URL.createObjectURL(blob);
    var dur=(Date.now()-gRecordStartTime)/1000;
    var now=new Date();
    var name='Rec_'+now.getHours().toString().padStart(2,'0')+
        now.getMinutes().toString().padStart(2,'0')+
        now.getSeconds().toString().padStart(2,'0');

    gRecordings.unshift({
        name:name,
        url:url,
        blob:blob,
        size_mb:blob.size/(1024*1024),
        duration_sec:dur,
        time:now.toLocaleTimeString()
    });
    gRecordedChunks=[];
    renderRecordingList();
}

// ---- 回放 ----
function playRecord(idx){
    if(idx<0||idx>=gRecordings.length)return;
    stopPlayStream();
    var r=gRecordings[idx];
    var video=document.getElementById('streamPlay');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    gCurrentPlayIdx=idx;
    video.src=r.url;
    video.style.display='block';
    ph.style.display='none';
    btn.style.display='inline-block';
    video.play().catch(function(e){console.error('Playback error:',e);});
    renderRecordingList();
}

function stopPlayback(){
    stopPlayStream();
    gCurrentPlayIdx=-1;
    var video=document.getElementById('streamPlay');
    var ph=document.getElementById('playbackPlaceholder');
    var btn=document.getElementById('btnStopPlay');
    video.pause();
    video.src='';
    video.style.display='none';
    ph.style.display='flex';
    btn.style.display='none';
    renderRecordingList();
}

function stopPlayStream(){
    var video=document.getElementById('streamPlay');
    if(video.src&&!video.paused){
        video.pause();
    }
}

function downloadRecord(idx){
    if(idx<0||idx>=gRecordings.length)return;
    var r=gRecordings[idx];
    var a=document.createElement('a');
    a.href=r.url;
    a.download=r.name+'.webm';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
}

// ---- 删除 ----
function deleteRecord(idx){
    if(idx<0||idx>=gRecordings.length)return;
    if(!confirm('删除录像 '+gRecordings[idx].name+' ?'))return;
    if(idx===gCurrentPlayIdx)stopPlayback();
    URL.revokeObjectURL(gRecordings[idx].url);
    gRecordings.splice(idx,1);
    renderRecordingList();
}

// ---- 格式化 ----
function formatDuration(sec){
    var m=Math.floor(sec/60);
    var s=Math.floor(sec%60);
    return (m<10?'0':'')+m+':'+(s<10?'0':'')+s;
}

function formatSize(mb){
    return mb>=1?mb.toFixed(1)+' MB':(mb*1024).toFixed(0)+' KB';
}

// ---- 渲染录像列表 ----
function renderRecordingList(){
    var list=document.getElementById('recList');
    if(gRecordings.length===0){
        list.innerHTML='<div class="empty-hint">暂无录像，点击「开始录像」进行录制</div>';
        return;
    }
    var h='';
    for(var i=0;i<gRecordings.length;i++){
        var r=gRecordings[i];
        var isPlaying=(i===gCurrentPlayIdx);
        h+='<div class="rec-item">'+
            '<span class="rec-name">'+(isPlaying?'&#9654; ':'')+r.name+'</span>'+
            '<span class="rec-meta">'+formatDuration(r.duration_sec)+' &middot; '+formatSize(r.size_mb)+'</span>'+
            '<button class="btn-play" onclick="playRecord('+i+')">播放</button>'+
            '<button class="btn-dl" onclick="downloadRecord('+i+')">下载</button>'+
            '<button class="btn-del" onclick="deleteRecord('+i+')">删除</button>'+
            '</div>';
    }
    list.innerHTML=h;
}

// ---- 启动 ----
startFrameCapture();
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
        except Exception:
            pass
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
            print(f"[PLAY] Stream error for {safe}: {e}")

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
        # 文件头: magic(6) + version(1) + frame_count placeholder(4)
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
            # 回写帧数到文件头偏移 7 处 (magic 6B + version 1B)
            f.seek(7)
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
        shared.record_frame_count += 1
    except Exception as e:
        print(f"[REC] Write error: {e}")
        stop_recording()


def get_local_ip():
    """获取本机局域网 IP"""
    return '192.168.43.130'


def start_web_server(port):
    """在守护线程中启动 HTTP 服务器（多线程，避免 MJPEG 流阻塞 API 请求）"""
    if ThreadingHTTPServer is not None:
        server = ThreadingHTTPServer(('0.0.0.0', port), StreamHandler)
        print(f"[WEB] Using multi-threaded HTTP server")
    else:
        server = HTTPServer(('0.0.0.0', port), StreamHandler)
        print(f"[WEB] Warning: single-threaded server, API may be blocked by MJPEG stream")
    ip = get_local_ip()
    print(f"[WEB] ========================================")
    print(f"[WEB]  HTTP server started on port {port}")
    if ip:
        print(f"[WEB]  Open in browser:")
        print(f"[WEB]    http://{ip}:{port}")
    else:
        print(f"[WEB]  Open in browser:")
        print(f"[WEB]    http://<MaixCAM2_IP>:{port}")
    print(f"[WEB] ========================================")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

# ============================================================
# 模型加载 + 硬件初始化
# ============================================================

model_path = "model_303178.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/mymodel/steelball/model_me/model_303178.mud"

detector = nn.YOLOv5(model=model_path)

cam = camera.Camera(detector.input_width(), detector.input_height(),
                    detector.input_format())
W, H = detector.input_width(), detector.input_height()
dis = display.Display()

# ---- LED 初始化 ----
LED_ON = True
if LED_ON:
    pin_name = "B25" if sys.device_id() == "maixcam2" else "B3"
    gpio_id  = "GPIOB25" if sys.device_id() == "maixcam2" else "GPIOB3"
    err.check_raise(pinmap.set_pin_function(pin_name, gpio_id), "set pin failed")
    led = gpio.GPIO(gpio_id, gpio.Mode.OUT)
    led.value(1)

# ---- UART 初始化 ----
ser = None
if ENABLE_UART:
    ser = uart.UART(UART_DEV, UART_BAUD)

# ---- 中心点 + 四分线 ----
CX = W // 2
CY = H // 2
Q1_X = W // 4
Q2_X = W // 2
Q3_X = W * 3 // 4

tracker = Tracker()

# ---- FPS ----
last_fps_ticks = time.ticks_ms()
frame_count = 0
fps_val = 0.0


# ============================================================
# Web 服务器启动（守护线程）
# ============================================================

if WEB_ENABLE:
    web_thread = threading.Thread(
        target=start_web_server, args=(WEB_PORT,),
        daemon=True, name="WebServer"
    )
    web_thread.start()
    print(f"[WEB] Web server thread started (port {WEB_PORT})")


# ============================================================
# 主循环（与 ball_to_center.py 完全一致 + JPEG 输出）
# ============================================================

while not app.need_exit():
    img = cam.read()
    frame_count += 1

    # ---- YOLO 推理（在画叠加线之前） ----
    raw_objs = detector.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)

    # ---- 背景叠加 ----
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
    detections = [(o.x + o.w // 2, o.y + o.h // 2, o.w, o.h, o.score)
                  for o in objs]

    # ---- 追踪器 ----
    confirmed_tracks = tracker.update(detections)

    # ---- 最近球 ----
    nearest = None
    if confirmed_tracks:
        nearest = min(confirmed_tracks,
                      key=lambda t: math.hypot(t.cx - CX, t.cy - CY))

    # ---- 绘制目标 ----
    draw_tracks = [nearest] if (SHOW_ONLY_NEAREST and nearest) else confirmed_tracks

    # ---- 巡线引导 ----
    ball_zone = calc_ball_zone(nearest, CX, W)
    has_ball = 1 if nearest else 0

    # ---- UART 发送 ----
    tx_count = 0
    if ser is not None:
        tx_cx = int(nearest.cx) if nearest else 0
        tx_cy = int(nearest.cy) if nearest else 0
        packet = build_ball_packet(has_ball, ball_zone, tx_cx, tx_cy)
        ser.write(packet)
        tx_count = len(packet)

    # ---- UART 接收 ----
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

        # 绿框
        img.draw_rect(bx, by, bw, bh, COLOR_GREEN, 2)

        # 红线：球心 -> 画面中心
        img.draw_line(int(t.cx), int(t.cy), CX, CY, COLOR_RED, 1)

        # 距离标注
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

    # ---- 状态叠加（左上角） ----
    img.draw_string(4, 4,  f"Z:{ball_zone:+d}", COLOR_WHITE)
    img.draw_string(4, 20, f"F:{fps_val:.0f}",  COLOR_WHITE)
    img.draw_string(4, 36, f"D:{has_ball}",     COLOR_WHITE)
    img.draw_string(4, 52, f"B:{len(confirmed_tracks)}", COLOR_WHITE)

    # ---- 屏幕显示 ----
    dis.show(img)

    # ---- Web 推流：转 JPEG 写入共享状态 ----
    if WEB_ENABLE:
        try:
            result = img.to_jpeg(quality=JPEG_QUALITY)
            # MaixPy: to_jpeg() 可能返回 Image 对象或 bytes，统一转 bytes
            if isinstance(result, bytes):
                jpeg = result
            else:
                jpeg = result.to_bytes()
            shared.update(jpeg, {
                "fps": fps_val,
                "ball_zone": ball_zone,
                "has_ball": has_ball,
                "track_count": len(confirmed_tracks),
                "frame_no": frame_count,
                "tx_bytes": tx_count,
                "rx_bytes": rx_count,
            })
            if frame_count == 1:
                print(f"[WEB] JPEG encoding OK, size={len(jpeg)} bytes")
            # 录像写入
            if RECORD_ENABLE:
                write_recording_frame(jpeg)
        except Exception as e:
            if frame_count <= 5:
                print(f"[WEB] JPEG encode failed: {e}")

    # ---- FPS ----
    if frame_count % 30 == 0:
        now = time.ticks_ms()
        elapsed = max(1, now - last_fps_ticks)
        fps_val = 30000.0 / elapsed
        last_fps_ticks = now
