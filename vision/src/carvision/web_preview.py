"""LAN preview of the real camera pipeline; ordinary serve remains read-only.

One acquisition/processing worker feeds all viewers. Slow clients never queue
camera frames. This is a local development server, not a public video service.
"""

import json
import secrets
import threading
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2

from .detection import YoloDetector
from .display import overlay
from .pipeline import Pipeline
from .sources import CameraSource
from .camera_assistance import TARGETS, camera_assistance


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>树莓派实时视觉</title>
<style>
body{margin:0;background:#111827;color:#e5e7eb;font:16px system-ui,sans-serif}
main{max-width:1100px;margin:auto;padding:24px}h1{font-size:24px;margin:0 0 8px}
p{color:#9ca3af;line-height:1.6}nav{display:flex;gap:8px;flex-wrap:wrap;margin:20px 0}
button{border:1px solid #475569;background:#1e293b;color:#e5e7eb;padding:10px 18px;border-radius:7px;cursor:pointer}
button.active{background:#2563eb;border-color:#60a5fa}.viewer{min-height:240px;background:#030712;border-radius:8px;overflow:hidden}
select{padding:10px;background:#1e293b;color:#e5e7eb;border:1px solid #475569;border-radius:7px}
img{display:block;width:100%;height:auto}img[hidden]{display:none}.metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;margin-top:18px}
.metric{padding:14px;background:#1e293b;border-radius:8px}.label{color:#94a3b8;font-size:13px}.value{font-size:20px;margin-top:6px}
#state{padding:12px;border-radius:7px;background:#78350f;margin-bottom:12px}#state.ok{background:#14532d}
small{color:#94a3b8;line-height:1.6}#error{color:#fca5a5;white-space:pre-wrap}
</style>
<main><h1>树莓派实时视觉</h1><p>原始画面持续更新，识别叠加画面按识别完成的速度更新。</p>
<label>画面来源：<select id="camera"><option value="primary">主摄像头</option></select></label><p id="camera-info">正在核对视频来源……</p>
<div id="camera-assist" hidden><label><input id="assist-enabled" type="checkbox" checked>主摄优先，第二路辅助观察</label>
<label>当前观察任务：<select id="assist-target"><option value="crosswalk">斑马线</option><option value="blue_board">蓝挡板</option><option value="traffic_light">红绿灯</option><option value="cone">锥桶</option><option value="qr">二维码</option></select></label>
<p id="assist-state">等待两路识别结果……</p></div>
<nav><button class="active" data-view="processed">识别叠加画面</button><button data-view="raw">原始画面</button><button data-view="mask">白线提取结果</button></nav>
<div id="state">等待摄像头首帧……</div><div class="viewer"><img id="video" alt="实时摄像头画面" hidden></div>
<div class="metrics"><div class="metric"><div class="label">道路检测</div><div class="value" id="lane">等待</div></div>
<div class="metric"><div class="label">目标偏差（正值在右）</div><div class="value" id="offset">—</div></div>
<div class="metric"><div class="label">本帧处理耗时</div><div class="value" id="process">—</div></div>
<div class="metric"><div class="label">预览更新速度</div><div class="value" id="fps">—</div></div>
<div class="metric"><div class="label">目标检测模型</div><div class="value" id="model">未加载</div></div>
<div class="metric"><div class="label">源帧编号</div><div class="value" id="frame">—</div></div></div>
<section id="race" hidden><h2>2026 比赛任务</h2><p id="race-state"></p><div class="metrics">
<div class="metric"><div class="label">蓝挡板</div><div id="race-board" class="value">未知</div></div>
<div class="metric"><div class="label">人行道</div><div id="race-crosswalk" class="value">未知</div></div>
<div class="metric"><div class="label">信号灯</div><div id="race-light" class="value">未知</div></div>
<div class="metric"><div class="label">锥桶候选</div><div id="race-cones" class="value">—</div></div></div>
<p id="race-rules"></p><p id="race-qr" style="white-space:pre-wrap;overflow-wrap:anywhere"></p></section>
<p id="error"></p><small>蓝/黄线：候选左右边界；绿线：中心路径；红点：预瞄点。白线提取图中白色为候选区域。<br>
道路无效时偏差为“—”。未加载比赛模型时不会显示锥桶等目标框。预览更新速度不等于车辆控制频率。</small></main>
<script>
const $=id=>document.getElementById(id);let view='processed',camera='primary',attached=false,knownCameras=false;
function connect(){ $('video').src='/stream.mjpg?camera='+encodeURIComponent(camera)+'&view='+view+'&t='+Date.now();attached=true; }
document.addEventListener('visibilitychange',()=>{if(document.hidden){$('video').removeAttribute('src');$('video').hidden=true;attached=false;}});
function changeCamera(next){if(next===camera)return;camera=next;$('camera').value=next;attached=false;$('video').hidden=true;$('video').removeAttribute('src');$('race').hidden=true;$('state').textContent='正在切换摄像头……';}
$('camera').onchange=()=>{$('assist-enabled').checked=false;changeCamera($('camera').value);$('assist-state').textContent='手动选择画面；勾选后恢复主摄优先配合。';};
async function loadCameras(signal){const response=await fetch('/api/cameras',{cache:'no-store',signal});if(!response.ok)throw new Error('无法核对相机');
 const sources=(await response.json()).sources;$('camera').replaceChildren();sources.forEach(source=>{const option=document.createElement('option');option.value=source.id;option.textContent=source.label;$('camera').appendChild(option);});
 $('camera').value=camera;knownCameras=true;$('camera-assist').hidden=sources.length<2;$('camera-info').textContent=sources.length>1?'主摄负责常规观察；第二路补充视野。':'当前配置一路视频。';}
async function updateAssistance(signal){
 if($('camera-assist').hidden||!$('assist-enabled').checked)return;
 const target=$('assist-target').value,response=await fetch('/api/assist?target='+encodeURIComponent(target),{cache:'no-store',signal});
 if(!response.ok)throw new Error('双摄配合状态不可用');const result=await response.json();
 if(document.hidden||!$('assist-enabled').checked||target!==$('assist-target').value)return;
 const names={primary:'主摄',secondary:'第二路'};
 let text=result.reason==='conflicting_traffic_light_observations'?'两路灯色不一致，等待确认':result.recognition_source?names[result.recognition_source]+'已识别当前目标':result.auxiliary_search?'主摄未确认目标，第二路辅助观察；两路继续识别':'等待有效识别结果';
 $('assist-state').textContent=text+'。主摄确认目标后自动返回。';changeCamera(result.recommended_view);
}
document.querySelectorAll('button[data-view]').forEach(b=>b.onclick=()=>{
 view=b.dataset.view;document.querySelectorAll('button[data-view]').forEach(x=>x.classList.toggle('active',x===b));
 $('video').removeAttribute('src');attached=false;
});
$('video').onerror=()=>{attached=false;};
async function poll(){
 const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),3000);
 try{
  if(document.hidden){$('video').removeAttribute('src');$('video').hidden=true;attached=false;return;}
  if(!knownCameras)await loadCameras(controller.signal);const selected=camera;
  const response=await fetch('/api/status?camera='+encodeURIComponent(selected),{cache:'no-store',signal:controller.signal});
  if(!response.ok)throw new Error('连接失败');const s=await response.json();const display=view==='raw'&&s.raw_preview?s.raw_preview:s;const ok=display.state==='ok';
  if(selected!==camera||document.hidden)return;
  $('state').className=ok?'ok':'';$('state').textContent=ok?(view==='raw'?'正在显示实时原始画面':'正在显示相机处理结果'):({starting:'等待摄像头首帧……',stale:'画面已超时，等待新帧',error:'摄像头或处理程序出错',stopped:'采集已停止'}[display.state]||display.state);
  $('video').hidden=!ok;$('error').textContent=s.error||'';
  if(ok&&!attached)connect();if(!ok){$('video').removeAttribute('src');attached=false;}
  const r=s.state==='ok'?s.result:null;$('lane').textContent=r?(r.lane.valid?'检测到双边界':'当前无有效道路'):'等待';
  $('offset').textContent=r&&r.lane.offset_normalized!==null?r.lane.offset_normalized.toFixed(3):'—';
  $('process').textContent=r?r.processing_ms.toFixed(1)+' ms':'—';$('fps').textContent=ok?display.preview_fps.toFixed(1)+' FPS':'—';
  $('model').textContent=r?({disabled:'未加载比赛模型',experimental_color_shape:'基础识别（待实测）',ok:'比赛模型'}[r.detector_status]||r.detector_status):'—';$('frame').textContent=view==='raw'&&s.raw_preview?(ok?s.raw_preview.frame_id:'—'):(r?r.frame_id:'—');
  const race=r&&r.race;$('race').hidden=!race;
  if(race){const text={present:'检测到',absent:'未检出',unknown:'未知',red:'红灯',yellow:'黄灯',green:'绿灯'};
   $('race-state').textContent='当前显示识别候选。完成距离标定、停稳确认和控制配置后，才能启用实车运行。';
   $('race-board').textContent=text[race.blue_board]||'未知';$('race-crosswalk').textContent=text[race.crosswalk]||'未知';
   $('race-light').textContent=text[race.traffic_light]||'未知';$('race-cones').textContent=r.detector_status==='disabled'?'未加载模型':race.cone_candidates;
   $('race-rules').textContent='人行道停车 '+race.crosswalk_hold_s+' 秒并播报；确认绿灯后通行；停车后 '+race.payment_window_s+' 秒内扫码支付 0.01 元。';
   $('race-qr').textContent=race.qr_values.length?'识别到二维码内容：'+race.qr_values.join('；'):'尚未识别到二维码';}
  try{await updateAssistance(controller.signal);}catch(e){$('assist-state').textContent='双摄配合状态暂不可用；当前画面继续显示。';}
 }catch(e){$('state').className='';$('state').textContent='与视觉服务断开，请检查程序与网络';$('video').hidden=true;$('video').removeAttribute('src');attached=false;
  $('race').hidden=true;['lane','offset','process','fps','model','frame'].forEach(id=>$(id).textContent='—');
 }finally{clearTimeout(timeout);setTimeout(poll,1000);}
}poll();
</script></html>"""


def preview_pipeline_config(config, preview_fps, stale_ms):
    """Let low-rate preview frames confirm cues without changing motion leases.

    Two scheduled frame periods plus processing jitter remain within the camera
    stale limit. Larger gaps still reset evidence, and the caller's race/control
    settings and configuration object remain untouched.
    """
    gap_ms = min(stale_ms, max(config.temporal.max_gap_ms, 2500 / preview_fps))
    return replace(config, temporal=replace(config.temporal, max_gap_ms=gap_ms))


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
        self.raw_enabled = False
        self.raw_sequence = 0
        self.raw_jpeg = None
        self.raw_frame_id = None
        self.raw_received_ms = None
        self.raw_updated = None
        self.raw_preview_fps = 0.0

    def publish_raw(self, jpeg, frame):
        """The latest raw frame is independent of recognition completion."""
        with self.condition:
            if self.state in ("error", "stopped"):
                return
            now = time.monotonic()
            if self.raw_updated is not None and now > self.raw_updated:
                rate = 1 / (now - self.raw_updated)
                self.raw_preview_fps = rate if not self.raw_preview_fps else .8*self.raw_preview_fps + .2*rate
            self.raw_enabled = True
            self.raw_updated = now
            self.raw_jpeg = jpeg
            self.raw_frame_id = frame.frame_id
            self.raw_received_ms = frame.receive_monotonic_ms
            self.raw_sequence += 1
            self.condition.notify_all()

    def _raw_status(self):
        age = None if self.raw_received_ms is None else max(0, time.monotonic()*1000-self.raw_received_ms)
        state = self.state if self.state in ("error", "stopped") else (
            "starting" if age is None else "stale" if age > self.stale_ms else "ok")
        return {"state": state, "preview_fps": self.raw_preview_fps,
                "host_frame_age_ms": age, "frame_id": self.raw_frame_id,
                "sequence": self.raw_sequence}

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
            self.raw_jpeg = self.raw_received_ms = self.raw_frame_id = None
            self.condition.notify_all()

    def _status(self):
        age = None if self.result is None else max(0, time.monotonic()*1000-self.result['receive_monotonic_ms'])
        state = "stale" if self.state == "ok" and age > self.stale_ms else self.state
        return {"state": state, "error": self.error, "preview_fps": self.preview_fps,
                "host_frame_age_ms": age, "result": self.result if state == "ok" else None,
                "sequence": self.sequence,
                "raw_preview": self._raw_status() if self.raw_enabled else None}

    def status(self):
        with self.condition:
            return self._status()

    def next_frame(self, sequence, view, timeout=2):
        with self.condition:
            def current_sequence():
                return self.raw_sequence if view == 'raw' and self.raw_enabled else self.sequence
            self.condition.wait_for(lambda: current_sequence() > sequence or self.state in ("error", "stopped"), timeout)
            raw = view == 'raw' and self.raw_enabled
            status = self._raw_status() if raw else self._status()
            current = current_sequence()
            jpeg = (self.raw_jpeg if raw else self.frames.get(view)) if status['state'] == 'ok' and current > sequence else None
            return current, jpeg, status['state']


class PreviewWorker:
    def __init__(self, state, config, args):
        self.state, self.config, self.args = state, config, args
        self.stop_event = threading.Event()
        self.source = None
        self.raw_thread = None
        self.thread = threading.Thread(target=self._run, name="vision-preview", daemon=True)

    def start(self):
        self.thread.start()

    def _encode(self, image):
        a = self.args
        h, w = image.shape[:2]
        if w > a.preview_width:
            image = cv2.resize(image, (a.preview_width, max(1, round(h*a.preview_width/w))), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality])
        if not ok:
            raise RuntimeError("preview JPEG encoding failed")
        return jpg.tobytes()

    def _run_raw(self):
        a = self.args
        fps = getattr(a, 'raw_preview_fps', None)
        fps = a.preview_fps if fps is None else fps
        next_publish = 0.0
        try:
            for frame in self.source:
                if self.stop_event.is_set():
                    break
                received = frame.receive_monotonic_ms / 1000
                if received < next_publish:
                    continue
                self.state.publish_raw(self._encode(frame.image), frame)
                next_publish = received + 1 / fps
        except Exception as exc:
            if not self.stop_event.is_set():
                self.state.finish(str(exc))

    def _run(self):
        try:
            a = self.args
            detector = YoloDetector(a.weights, self.config.yolo) if a.weights else None
            if getattr(a, 'classic_candidates', False):
                if a.weights:
                    raise ValueError("choose race-trained weights or experimental classic candidates, not both")
                from .classic_detection import ClassicRaceDetector
                detector = ClassicRaceDetector()
            preview_config = preview_pipeline_config(self.config, a.preview_fps, a.stale_ms)
            pipeline = Pipeline(preview_config, detector)
            monitor = None
            if getattr(a, 'race_config', None):
                from .race import load_race_config
                from .race_monitor import RaceMonitor
                monitor = RaceMonitor(load_race_config(a.race_config))
            self.source = CameraSource(a.camera, a.width, a.height, a.fps,
                                       fourcc=getattr(a,'camera_format','MJPG'), decode_fps=a.fps)
            if self.source.identity:
                self.state.identity = self.source.identity
            self.state.capture = self.source.reported
            self.raw_thread = threading.Thread(target=self._run_raw, name="raw-preview", daemon=True)
            self.raw_thread.start()
            last_publish = 0.0
            for frame in self.source:
                if self.stop_event.is_set():
                    break
                if time.monotonic()-last_publish < 1/a.preview_fps:
                    continue
                result, mask = pipeline.process(frame, live=True)
                views = {"processed": overlay(frame.image, result, self.config.lane.roi_top), "mask": mask}
                encoded = {name: self._encode(image) for name, image in views.items()}
                result_data = result.to_dict()
                if monitor:
                    result_data['race'] = monitor.inspect(frame.image, result)
                self.state.publish(encoded, result_data)
                last_publish = time.monotonic()
        except Exception as exc:
            if not self.stop_event.is_set():
                self.state.finish(str(exc))
        finally:
            self.stop_event.set()
            if self.source:
                self.source.close()
            if self.raw_thread:
                self.raw_thread.join(timeout=1)
            if self.state.status()['state'] != 'error':
                self.state.finish()

    def close(self):
        self.stop_event.set()
        self.state.finish()
        if self.source:
            self.source.close()
        self.thread.join(timeout=2)


def make_server(address, state, secondary=None, drive=None):
    states = {'primary': state}
    if secondary is not None:
        states['secondary'] = secondary
    drive_key = secrets.token_hex(24) if drive else None
    drive_session = secrets.token_hex(12) if drive else None
    page = PAGE
    if drive:
        from .manual_controls import HTML, SCRIPT, STYLE, AUX_SCRIPT
        page = page.replace("let view='processed'", "let view='raw'")
        page = page.replace('class="active" data-view="processed"', 'data-view="processed"')
        page = page.replace('<button data-view="raw">', '<button class="active" data-view="raw">')
        page = page.replace('id="assist-enabled" type="checkbox" checked', 'id="assist-enabled" type="checkbox"')
        page = page.replace('<title>树莓派实时视觉</title>', '<title>智能车遥控驾驶台</title>')
        page = page.replace('</style>', '</style>'+STYLE, 1)
        page = page.replace('<h1>树莓派实时视觉</h1>', '<h1>智能车遥控驾驶台</h1>')
        page = page.replace('原始画面持续更新，识别叠加画面按识别完成的速度更新。',
                            '电脑键盘控制 · 双路实时画面 · 速度指令调节')
        page = page.replace('<label>画面来源：', '<div class="driving-layout"><section class="camera-zone"><label>驾驶画面：')
        aux = ('<section id="aux-camera" class="aux-camera" hidden><h3 id="aux-title">第二路画面</h3>'
               '<p id="aux-state">正在连接摄像头……</p><div class="viewer">'
               '<img id="aux-video" alt="第二路实时摄像头画面" hidden></div></section>')
        page = page.replace('<div class="metrics"><div class="metric"><div class="label">道路检测',
                            aux+'</section>'+HTML+'</div><details class="vision-details"><summary>图像分析信息</summary>'
                            '<div class="metrics"><div class="metric"><div class="label">道路检测', 1)
        page = page.replace('</small></main>', '</small></details></main>')
        script = SCRIPT.replace('__DRIVE_KEY__', json.dumps(drive_key)).replace('__DRIVE_SESSION__', json.dumps(drive_session))
        page = page.replace('</html>', script+AUX_SCRIPT+'</html>')
    class Handler(BaseHTTPRequestHandler):
        # Reuse the control connection across heartbeats on cellular links.
        protocol_version = 'HTTP/1.1'
        disable_nagle_algorithm = True
        timeout = 5

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
            if self.close_connection:
                self.send_header('Connection', 'close')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            try:
                parsed = urlsplit(self.path)
                if parsed.path == '/':
                    return self.send_body(page.encode('utf-8'), 'text/html; charset=utf-8')
                if parsed.path == '/api/drive/status' and drive:
                    return self.send_body(json.dumps({**drive.status(), 'control_session_id': drive_session}, ensure_ascii=False).encode(), 'application/json')
                if parsed.path == '/api/cameras':
                    rows = [{'id': key, 'label': ('主摄像头' if key == 'primary' else '第二路摄像头') +
                             (' · ' + getattr(value, 'identity', {}).get('name', '') if getattr(value, 'identity', None) else ''),
                             'depth_stream_verified': False} for key, value in states.items()]
                    return self.send_body(json.dumps({'sources': rows}, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8')
                if parsed.path == '/api/assist':
                    target = parse_qs(parsed.query).get('target', ['crosswalk'])[0]
                    if target not in TARGETS:
                        return self.send_body(b'Unknown observation target', 'text/plain', 400)
                    assistance = camera_assistance(state.status(), secondary.status() if secondary is not None else None, target)
                    return self.send_body(json.dumps(assistance, ensure_ascii=False, allow_nan=False).encode('utf-8'), 'application/json; charset=utf-8')
                camera_id = parse_qs(parsed.query).get('camera', ['primary'])[0]
                if camera_id not in states:
                    return self.send_body(b'Unknown camera; no fallback source', 'text/plain', 400)
                selected_state = states[camera_id]
                if parsed.path == '/api/status':
                    status = {**selected_state.status(), 'camera_id': camera_id, 'camera_identity': getattr(selected_state, 'identity', None),
                              'camera_capture':getattr(selected_state,'capture',None)}
                    return self.send_body(json.dumps(status, ensure_ascii=False, allow_nan=False).encode('utf-8'), 'application/json; charset=utf-8')
                if parsed.path not in ('/stream.mjpg', '/frame.jpg'):
                    return self.send_body(b'Not found', 'text/plain', 404)
                view = parse_qs(parsed.query).get('view', ['processed'])[0]
                if view not in ('raw', 'processed', 'mask'):
                    return self.send_body(b'Unknown view', 'text/plain', 400)
                if parsed.path == '/frame.jpg':
                    _, jpeg, _ = selected_state.next_frame(-1, view, timeout=0)
                    return self.send_body(jpeg or b'No fresh camera frame', 'image/jpeg' if jpeg else 'text/plain', 200 if jpeg else 503)
                # Multipart video ends by closing the socket; JSON responses
                # above have a length and can keep their connection alive.
                self.close_connection = True
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store, max-age=0')
                self.send_header('Connection', 'close')
                self.end_headers()
                sequence = -1
                while True:
                    new_sequence, jpeg, status = selected_state.next_frame(sequence, view)
                    if status in ('error', 'stopped'):
                        break
                    if jpeg is None:
                        sequence = new_sequence
                        continue
                    sequence = new_sequence
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    self.wfile.flush()
            except (OSError, ConnectionError):
                self.close_connection = True
                return

        def do_POST(self):
            if drive is None or not urlsplit(self.path).path.startswith('/api/drive/'):
                self.close_connection = True
                return self.send_body(b'Not found', 'text/plain', 404)
            origin = self.headers.get('Origin')
            if (self.headers.get('X-Drive-Key') != drive_key
                    or origin and origin != 'http://'+self.headers.get('Host', '')
                    or self.headers.get('Content-Type') != 'application/json'):
                # The rejected request body has not been consumed.
                self.close_connection = True
                error = {'error': '控制页面已过期或来源不匹配，请刷新页面后重新启用', 'code': 'control_page_rejected'}
                return self.send_body(json.dumps(error, ensure_ascii=False).encode(), 'application/json', 403)
            try:
                if self.headers.get('Transfer-Encoding') is not None:
                    raise ValueError('control requests require Content-Length')
                if len(self.headers.get_all('Content-Length', [])) != 1:
                    raise ValueError('control requests require one Content-Length')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('control body must be 1..2048 bytes')
                payload = json.loads(self.rfile.read(size))
                result = drive.request(urlsplit(self.path).path.rsplit('/', 1)[-1], payload)
                return self.send_body(json.dumps(result, ensure_ascii=False).encode(), 'application/json')
            except (ValueError, OSError) as exc:
                self.close_connection = True
                return self.send_body(json.dumps({'error': str(exc)}).encode(), 'application/json', 400)

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server


def serve(args, config, drive=None):
    raw_fps = getattr(args, 'raw_preview_fps', None)
    secondary_raw_fps = getattr(args, 'secondary_raw_preview_fps', None)
    if not (1 <= args.port <= 65535 and 1 <= args.preview_fps <= 60 and args.preview_width >= 160
            and 20 <= args.jpeg_quality <= 95 and args.stale_ms > 0
            and (raw_fps is None or 1 <= raw_fps <= 60)
            and (secondary_raw_fps is None or 1 <= secondary_raw_fps <= 60)):
        raise ValueError("invalid port, preview FPS/width, JPEG quality or stale interval")
    from copy import copy
    aux = getattr(args, 'aux_camera', None)
    if aux is not None:
        from .cameras import device_path
        if aux == args.camera or device_path(aux).resolve() == device_path(args.camera).resolve():
            raise ValueError('both sources select the same camera node; choose two distinct capture nodes')
    state = PreviewState(args.stale_ms)
    secondary = PreviewState(args.stale_ms) if aux is not None else None
    server = make_server((args.host, args.port), state, secondary, drive)
    workers = [PreviewWorker(state, config, args)]
    if secondary:
        second_args = copy(args)
        second_args.camera = aux
        if secondary_raw_fps is not None:
            second_args.raw_preview_fps = secondary_raw_fps
        workers.append(PreviewWorker(secondary, config, second_args))
    try:
        for worker in workers:
            worker.start()
        print(f"Live camera preview listening on {args.host}:{args.port}", flush=True)
        print(f"Open http://<device-IP>:{args.port} on a computer on the same network. Ctrl+C stops capture.", flush=True)
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        if drive:
            drive.close()
        for worker in workers:
            worker.close()
        server.server_close()
    return {"status": "stopped", "processing_worker_stopped": all(not worker.thread.is_alive() for worker in workers)}
