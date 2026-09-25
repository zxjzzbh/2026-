"""Read-only LAN preview of the real camera pipeline; no desktop required.

One acquisition/processing worker feeds all viewers. Slow clients never queue
camera frames. This is a local development server, not a public video service.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2

from .detection import YoloDetector
from .display import overlay
from .pipeline import Pipeline
from .sources import CameraSource


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>树莓派实时视觉</title>
<style>
body{margin:0;background:#111827;color:#e5e7eb;font:16px system-ui,sans-serif}
main{max-width:1100px;margin:auto;padding:24px}h1{font-size:24px;margin:0 0 8px}
p{color:#9ca3af;line-height:1.6}nav{display:flex;gap:8px;flex-wrap:wrap;margin:20px 0}
button{border:1px solid #475569;background:#1e293b;color:#e5e7eb;padding:10px 18px;border-radius:7px;cursor:pointer}
button.active{background:#2563eb;border-color:#60a5fa}.viewer{min-height:240px;background:#030712;border-radius:8px;overflow:hidden}
img{display:block;width:100%;height:auto}img[hidden]{display:none}.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;margin-top:18px}
.metric{padding:14px;background:#1e293b;border-radius:8px}.label{color:#94a3b8;font-size:13px}.value{font-size:20px;margin-top:6px}
#state{padding:12px;border-radius:7px;background:#78350f;margin-bottom:12px}#state.ok{background:#14532d}
small{color:#94a3b8;line-height:1.6}#error{color:#fca5a5;white-space:pre-wrap}
</style>
<main><h1>树莓派实时视觉</h1><p>相机画面由运行本程序的设备采集，识别完成后发送到当前浏览器。</p>
<nav><button class="active" data-view="processed">识别叠加画面</button><button data-view="raw">原始画面</button><button data-view="mask">白线提取结果</button></nav>
<div id="state">等待摄像头首帧……</div><div class="viewer"><img id="video" alt="实时摄像头画面" hidden></div>
<div class="metrics"><div class="metric"><div class="label">道路检测</div><div class="value" id="lane">等待</div></div>
<div class="metric"><div class="label">目标偏差（正值在右）</div><div class="value" id="offset">—</div></div>
<div class="metric"><div class="label">本帧处理耗时</div><div class="value" id="process">—</div></div>
<div class="metric"><div class="label">预览更新速度</div><div class="value" id="fps">—</div></div>
<div class="metric"><div class="label">目标检测模型</div><div class="value" id="model">未加载</div></div>
<div class="metric"><div class="label">源帧编号</div><div class="value" id="frame">—</div></div></div>
<p id="error"></p><small>蓝/黄线：候选左右边界；绿线：中心路径；红点：预瞄点。白线提取图中白色为候选区域。<br>
道路无效时偏差为“—”。未加载比赛模型时不会显示锥桶等目标框。预览更新速度不等于车辆控制频率。</small></main>
<script>
const $=id=>document.getElementById(id);let view='processed',attached=false;
function connect(){ $('video').src='/stream.mjpg?view='+view+'&t='+Date.now();attached=true; }
document.querySelectorAll('button[data-view]').forEach(b=>b.onclick=()=>{
 view=b.dataset.view;document.querySelectorAll('button').forEach(x=>x.classList.toggle('active',x===b));
 $('video').removeAttribute('src');attached=false;
});
$('video').onerror=()=>{attached=false;};
async function poll(){
 const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),3000);
 try{
  const response=await fetch('/api/status',{cache:'no-store',signal:controller.signal});
  if(!response.ok)throw new Error('连接失败');const s=await response.json();const ok=s.state==='ok';
  $('state').className=ok?'ok':'';$('state').textContent=ok?'正在显示相机处理结果':({starting:'等待摄像头首帧……',stale:'画面已超时，等待新帧',error:'摄像头或处理程序出错',stopped:'采集已停止'}[s.state]||s.state);
  $('video').hidden=!ok;$('error').textContent=s.error||'';
  if(ok&&!attached)connect();if(!ok){$('video').removeAttribute('src');attached=false;}
  const r=ok?s.result:null;$('lane').textContent=r?(r.lane.valid?'检测到双边界':'当前无有效道路'):'等待';
  $('offset').textContent=r&&r.lane.offset_normalized!==null?r.lane.offset_normalized.toFixed(3):'—';
  $('process').textContent=r?r.processing_ms.toFixed(1)+' ms':'—';$('fps').textContent=ok?s.preview_fps.toFixed(1)+' FPS':'—';
  $('model').textContent=r?(r.detector_status==='disabled'?'未加载比赛模型':r.detector_status):'—';$('frame').textContent=r?r.frame_id:'—';
 }catch(e){$('state').className='';$('state').textContent='与视觉服务断开，请检查程序与网络';$('video').hidden=true;$('video').removeAttribute('src');attached=false;
  ['lane','offset','process','fps','model','frame'].forEach(id=>$(id).textContent='—');
 }finally{clearTimeout(timeout);setTimeout(poll,700);}
}poll();
</script></html>"""


class PreviewState:
    def __init__(self, stale_ms=1500):
        self.condition = threading.Condition()
        self.state = "starting"
        self.error = None
        self.sequence = 0
        self.frames = {}
        self.result = None
        self.updated = None
        self.preview_fps = 0.0
        self.stale_ms = stale_ms

    def publish(self, frames, result):
        with self.condition:
            if self.state in ("error", "stopped"):
                return
            now = time.monotonic()
            if self.updated is not None and now > self.updated:
                rate = 1/(now-self.updated)
                self.preview_fps = rate if not self.preview_fps else .8*self.preview_fps+.2*rate
            self.updated = now
            self.frames, self.result = frames, result
            self.sequence += 1
            self.state = "ok"
            self.condition.notify_all()

    def finish(self, error=None):
        with self.condition:
            self.state = "error" if error else "stopped"
            self.error = error
            self.frames, self.result = {}, None
            self.condition.notify_all()

    def _status(self):
        age = None if self.result is None else max(0, time.monotonic()*1000-self.result['receive_monotonic_ms'])
        state = "stale" if self.state == "ok" and age > self.stale_ms else self.state
        return {"state": state, "error": self.error, "preview_fps": self.preview_fps,
                "host_frame_age_ms": age, "result": self.result if state == "ok" else None,
                "sequence": self.sequence}

    def status(self):
        with self.condition:
            return self._status()

    def next_frame(self, sequence, view, timeout=2):
        with self.condition:
            self.condition.wait_for(lambda: self.sequence > sequence or self.state in ("error", "stopped"), timeout)
            status = self._status()
            jpeg = self.frames.get(view) if status['state'] == 'ok' and self.sequence > sequence else None
            return self.sequence, jpeg, status['state']


class PreviewWorker:
    def __init__(self, state, config, args):
        self.state, self.config, self.args = state, config, args
        self.stop_event = threading.Event()
        self.source = None
        self.thread = threading.Thread(target=self._run, name="vision-preview", daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            a = self.args
            detector = YoloDetector(a.weights, self.config.yolo) if a.weights else None
            pipeline = Pipeline(self.config, detector)
            self.source = CameraSource(a.camera, a.width, a.height, a.fps)
            last_publish = 0.0
            for frame in self.source:
                if self.stop_event.is_set():
                    break
                if time.monotonic()-last_publish < 1/a.preview_fps:
                    continue
                result, mask = pipeline.process(frame, live=True)
                views = {"raw": frame.image, "processed": overlay(frame.image, result, self.config.lane.roi_top), "mask": mask}
                encoded = {}
                for name, image in views.items():
                    h, w = image.shape[:2]
                    if w > a.preview_width:
                        image = cv2.resize(image, (a.preview_width, max(1, round(h*a.preview_width/w))), interpolation=cv2.INTER_AREA)
                    ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality])
                    if not ok:
                        raise RuntimeError("preview JPEG encoding failed")
                    encoded[name] = jpg.tobytes()
                self.state.publish(encoded, result.to_dict())
                last_publish = time.monotonic()
        except Exception as exc:
            if not self.stop_event.is_set():
                self.state.finish(str(exc))
        finally:
            if self.source:
                self.source.close()
            if self.state.status()['state'] != 'error':
                self.state.finish()

    def close(self):
        self.stop_event.set()
        self.state.finish()
        if self.source:
            self.source.close()
        self.thread.join(timeout=2)


def make_server(address, state):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(5)

        def log_message(self, *args):
            pass

        def send_body(self, content, mime, status=200):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store, max-age=0')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            try:
                parsed = urlsplit(self.path)
                if parsed.path == '/':
                    return self.send_body(PAGE.encode('utf-8'), 'text/html; charset=utf-8')
                if parsed.path == '/api/status':
                    return self.send_body(json.dumps(state.status(), ensure_ascii=False, allow_nan=False).encode('utf-8'), 'application/json; charset=utf-8')
                if parsed.path not in ('/stream.mjpg', '/frame.jpg'):
                    return self.send_body(b'Not found', 'text/plain', 404)
                view = parse_qs(parsed.query).get('view', ['processed'])[0]
                if view not in ('raw', 'processed', 'mask'):
                    return self.send_body(b'Unknown view', 'text/plain', 400)
                if parsed.path == '/frame.jpg':
                    _, jpeg, _ = state.next_frame(-1, view, timeout=0)
                    return self.send_body(jpeg or b'No fresh camera frame', 'image/jpeg' if jpeg else 'text/plain', 200 if jpeg else 503)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store, max-age=0')
                self.end_headers()
                sequence = -1
                while True:
                    new_sequence, jpeg, status = state.next_frame(sequence, view)
                    if status in ('error', 'stopped'):
                        break
                    if jpeg is None:
                        sequence = new_sequence
                        continue
                    sequence = new_sequence
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    self.wfile.flush()
            except (OSError, ConnectionError):
                return

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def serve(args, config):
    if not (1 <= args.port <= 65535 and 1 <= args.preview_fps <= 60 and args.preview_width >= 160
            and 20 <= args.jpeg_quality <= 95 and args.stale_ms > 0):
        raise ValueError("invalid port, preview FPS/width, JPEG quality or stale interval")
    state = PreviewState(args.stale_ms)
    server = make_server((args.host, args.port), state)
    worker = PreviewWorker(state, config, args)
    try:
        worker.start()
        print(f"Live camera preview listening on {args.host}:{args.port}", flush=True)
        print(f"Open http://<device-IP>:{args.port} on a computer on the same network. Ctrl+C stops capture.", flush=True)
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        worker.close()
        server.server_close()
    return {"status": "stopped", "processing_worker_stopped": not worker.thread.is_alive()}
