"""Replay real dual-camera recordings; no control endpoints. Author: 13246."""
import base64
import json
from functools import lru_cache
from pathlib import Path
from urllib.parse import parse_qs

from .cone_recording import RUN_ID, encoded


def rows(path):
    if not path.exists():return
    with path.open(encoding='utf8') as handle:
        for line in handle:
            try:yield json.loads(line)
            except ValueError:continue  # An interrupted final line is not a frame.


def frame_key(camera, row):
    return (camera,row.get('frame_id'),row.get('captured_s',row.get('received_s')))


def build_replay_data(folder, origin):
    folder=Path(folder);frames=list(rows(folder/'frames.jsonl'));events=[];observations={};marks=[]
    previous=None;previous_steer=None
    for event in rows(folder/'events.jsonl'):
        data=event['data'];kind=event['kind'];t=event['recorded_s']-origin
        if kind=='observation':
            for camera,row in data.get('cameras',{}).items():
                observations.setdefault(frame_key(camera,row),{'computed_t':t,
                    'tracks':row.get('tracks',[]),'road':row.get('road_context',{})})
        elif kind=='decision':
            intent=data.get('intent') or {};progress=data.get('progress') or {};nav=data.get('navigation') or {}
            execution=data.get('execution') or {};command=execution.get('command') or {}
            sent=command.get('sent_s',data.get('command_dispatched_s',event['recorded_s']))
            item={'t':sent-origin,'kind':kind,'intent':intent,'progress':progress,
                'execution':execution,'camera_frames':data.get('camera_frames',{}),
                'source_camera':nav.get('source_camera'),'navigation_reason':nav.get('reason'),
                'decision_started_t':data.get('now_s',sent)-origin,
                'perception_finished_t':data.get('perception_finished_s',sent)-origin}
            events.append(item)
            guidance=progress.get('image_guidance') or {}
            key=(guidance.get('phase',intent.get('phase')),guidance.get('target_id',progress.get('target_id')),
                 progress.get('passed_count'))
            labels=[]
            if key!=previous:labels.append('阶段 %s · 目标 %s · 通过估计 %s'%key)
            steer=command.get('steering_us',intent.get('steering_us'))
            if steer==1610 and previous_steer not in (None,1610):labels.append('发出转向回中请求')
            if labels:marks.append({'t':item['t'],'label':'；'.join(labels)})
            previous=key;previous_steer=steer
        else:
            events.append({'t':t,'kind':kind,'data':data})
            if kind=='stop':marks.append({'t':t,'label':'结束：'+str(data.get('reason',''))})
    frames.sort(key=lambda r:r['received_s'])
    for i,frame in enumerate(frames):
        frame.update(i=i,t=frame['received_s']-origin,
                     result=observations.get(frame_key(frame['camera'],frame)))
    events.sort(key=lambda e:e['t']);marks.sort(key=lambda e:e['t'])
    return {'id':folder.name,'origin_s':origin,'frames':frames,'events':events,'milestones':marks,
        'duration':max([0.]+[f['t'] for f in frames]+[e['t'] for e in events]),
        'timestamp_semantics':'host receive time; output is worker dispatch, not measured wheel angle'}


@lru_cache(maxsize=4)
def load_replay(folder):
    return json.loads((Path(folder)/'replay-data.json').read_text(encoding='utf8'))


def read_jpeg(folder, frame):
    path=Path(folder)/'frames.mjpg';offset=frame['offset'];length=frame['length']
    if type(offset) is not int or type(length) is not int or offset<0 or not 0<length<=1024*1024:
        raise ValueError('Invalid frame index')
    if offset+length>path.stat().st_size:raise ValueError('Truncated frame archive')
    with path.open('rb') as handle:handle.seek(offset);return handle.read(length)


def response(root, url, kind):
    try:
        query=parse_qs(url.query);name=query.get('run',[''])[0]
        if not RUN_ID.fullmatch(name):raise ValueError('Invalid recording ID')
        root=Path(root).resolve();folder=(root/name).resolve()
        if folder.parent!=root:raise ValueError('Invalid recording path')
        summary=json.loads((folder/'recording.json').read_text(encoding='utf8'))
        if not summary.get('finalized'):return b'Recording is still closing','text/plain',409
        if kind=='html':return REPLAY_HTML.encode('utf8'),'text/html; charset=utf-8',200
        data=load_replay(str(folder))
        if kind=='data':return encoded({**data,'recording':summary}),'application/json; charset=utf-8',200
        index=int(query.get('i',['-1'])[0])
        if not 0<=index<len(data['frames']):raise ValueError('Invalid frame number')
        return read_jpeg(folder,data['frames'][index]),'image/jpeg',200
    except (OSError,ValueError,KeyError,IndexError) as exc:
        return str(exc).encode('utf8'),'text/plain; charset=utf-8',404


def export_portable(folder, output):
    """Explicit offline export; streams each JPEG instead of loading the archive."""
    folder=Path(folder);output=Path(output)
    if output.suffix.lower()!='.html':raise ValueError('Replay output must be an .html file')
    summary=json.loads((folder/'recording.json').read_text(encoding='utf8'))
    if not summary.get('finalized'):raise ValueError('Wait for recording finalization')
    data=load_replay(str(folder.resolve()))
    def dump(value):return json.dumps(value,ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    temp=output.with_name(output.name+'.tmp')
    try:
        with temp.open('w',encoding='utf8') as dest:
            before,after=REPLAY_HTML.split('<!--PORTABLE_DATA-->')
            dest.write(before+'<script type="application/json" id="portable-data">')
            dest.write(dump({**data,'recording':summary,'frames':[]})[:-1])
            # Replace the empty frame array with a distinct field consumed below.
            dest.write(',"embedded_frames":[')
            for i,frame in enumerate(data['frames']):
                if i:dest.write(',')
                dest.write(dump({**frame,'src':'data:image/jpeg;base64,'+base64.b64encode(read_jpeg(folder,frame)).decode('ascii')}))
            dest.write(']}</script>'+after)
        temp.replace(output)
    except Exception:
        temp.unlink(missing_ok=True);raise
    return output


REPLAY_HTML=r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>连续绕行 · 实拍回放</title>
<style>*{box-sizing:border-box}body{font:16px system-ui;margin:24px;background:#f4f6fa;color:#172033}main{max-width:1200px;margin:auto}button,select,a{padding:9px;margin:3px}select{max-width:100%}input{width:100%}.views{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}section{min-width:0;background:white;padding:14px;border-radius:10px;margin:12px 0}canvas{width:100%;background:#222}pre{white-space:pre-wrap;overflow-wrap:anywhere}#summary{color:#8b4300}@media(max-width:650px){.views{grid-template-columns:minmax(0,1fr)}body{margin:10px}}</style>
<main><h1>连续绕行 · 实拍回放</h1><p>两路实际 JPEG 与控制日志。转向值是向输出进程发送的请求，不代表测得的前轮角度。时间轴采用本机接收时间。</p>
<section><div id="summary">载入记录…</div><button id="play">播放</button><button id="prev">上一帧</button><button id="next">下一帧</button><select id="speed"><option>.25</option><option>.5</option><option selected>1</option><option>2</option></select>倍速 <select id="marks"><option value="">跳到事件</option></select><input id="seek" type="range" min="0" step=".001" value="0"><div id="time"></div></section>
<div class="views"><section><h2>第一路</h2><canvas id="primary"></canvas><p id="primary-info"></p></section><section><h2>第二路</h2><canvas id="secondary"></canvas><p id="secondary-info"></p></section></div><section><h2>此时的控制记录</h2><pre id="event"></pre></section></main>
<!--PORTABLE_DATA--><script>
'use strict';let data,t=0,playing=false,last=0;const $=id=>document.getElementById(id),shown={},generation={};
const run=new URLSearchParams(location.search).get('run');
function chooseFrame(frames,time){let lo=0,hi=frames.length;while(lo<hi){let m=(lo+hi)>>1;if(frames[m].t<=time)lo=m+1;else hi=m}return lo?frames[lo-1]:null}
function paint(camera,f){const canvas=$(camera);generation[camera]=(generation[camera]||0)+1;const token=generation[camera];const context=canvas.getContext('2d');context.clearRect(0,0,canvas.width,canvas.height);if(!f)return;
 const img=new Image();img.onload=()=>{if(generation[camera]!==token)return;canvas.width=img.naturalWidth;canvas.height=img.naturalHeight;context.drawImage(img,0,0);const r=f.result;if(!r)return;
 context.lineWidth=2;for(const p of [r.road.left_inner||[],r.road.right_inner||[]]){context.strokeStyle='#12fba0';context.beginPath();p.forEach((v,i)=>i?context.lineTo(...v):context.moveTo(...v));context.stroke()}
 for(const track of r.tracks){if(!track.visible||!track.bbox_xyxy)continue;const [x,y,x2,y2]=track.bbox_xyxy;context.strokeStyle='#ffb53b';context.strokeRect(x,y,x2-x,y2-y);context.fillStyle='#ffb53b';context.font='13px sans-serif';context.fillText(track.id,x,Math.max(15,y-3))}};
 img.onerror=()=>{if(generation[camera]===token)$(camera+'-info').textContent+=' · 图片读取失败'};
 img.src=f.src||('/api/cone-latest/recorded-frame.jpg?run='+encodeURIComponent(data.id)+'&i='+f.i)
}
function render(){if(!data)return;t=Math.max(0,Math.min(data.duration,t));$('seek').value=t;$('time').textContent=t.toFixed(3)+' / '+data.duration.toFixed(3)+' 秒';
 for(const camera of ['primary','secondary']){const f=chooseFrame(data.views[camera],t);if(shown[camera]!==f){shown[camera]=f;paint(camera,f)}
 $(camera+'-info').textContent=f?'帧 '+f.frame_id+' · 画面已过 '+((t-f.t)*1000).toFixed(0)+' ms · '+(f.result?'本帧识别结果，计算于 '+f.result.computed_t.toFixed(3)+' s':'本帧未参与识别，未叠加其他帧结果'):'此时尚无已记录画面';}
 const e=chooseFrame(data.events,t);$('event').textContent=e?JSON.stringify(e,null,2):'此时尚无控制记录';}
function step(direction){playing=false;let f=direction>0?data.frames.find(f=>f.t>t+.000001):[...data.frames].reverse().find(f=>f.t<t-.000001);if(f)t=f.t;render()}
function animate(now){if(playing){t+=(now-last)/1000*Number($('speed').value);if(t>=data.duration)playing=false;render()}last=now;requestAnimationFrame(animate)}
async function init(){const embedded=$('portable-data');if(embedded){data=JSON.parse(embedded.textContent);data.frames=data.embedded_frames||data.frames}else{const response=await fetch('/api/cone-latest/recording.json?run='+encodeURIComponent(run));if(!response.ok)throw Error(await response.text());data=await response.json()}
 data.views=Object.fromEntries(['primary','secondary'].map(c=>[c,data.frames.filter(f=>f.camera===c)]));$('seek').max=data.duration;
 const r=data.recording;$('summary').textContent='原始帧 '+data.frames.length+'；缺失记录 '+JSON.stringify(r.dropped||{})+'；'+(r.error||'记录结束')+'。接收帧完整性：'+(r.complete?'本次记录范围内完整':'存在缺失或错误，不能当作完整视频');
 for(const mark of data.milestones){const option=document.createElement('option');option.value=mark.t;option.textContent=mark.t.toFixed(2)+'s '+mark.label;$('marks').append(option)}
 $('play').onclick=()=>{if(t>=data.duration)t=0;playing=!playing;$('play').textContent=playing?'暂停':'播放'};$('prev').onclick=()=>step(-1);$('next').onclick=()=>step(1);$('seek').oninput=e=>{playing=false;t=Number(e.target.value);render()};$('marks').onchange=e=>{if(e.target.value==='')return;playing=false;t=Number(e.target.value);render()};render();requestAnimationFrame(animate)}
init().catch(error=>{$('summary').textContent='回放载入失败：'+error.message});
</script></html>'''
