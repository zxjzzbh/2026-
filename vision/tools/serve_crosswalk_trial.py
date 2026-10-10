"""Run the current car console with a finite crosswalk test extension.

Requires the CURRENT deployed serve_boot_test.py, not the older GitHub copy.
No motion occurs on startup; existing ESC preparation and explicit trial start
remain necessary. Uses the existing driver's camera/UART ownership.
"""
import argparse
import importlib.util
import sys
from pathlib import Path


CONTROL_HTML = '''<section class="drive-panel" id="crosswalk-test">
<h3>斑马线停车测试</h3>
<p id="crosswalk-test-state">等待实测停车参考</p>
<p id="crosswalk-reference-state"></p>
<select id="crosswalk-path-mode"><option value="straight">直线前进搜索斑马线（停车短测）</option>
<option value="lane">沿已识别赛道线接近</option></select>
<label><input type="checkbox" id="crosswalk-field-ready">场地空旷，有人可关闭电调</label>
<button id="crosswalk-start" type="button">开始斑马线短测</button>
<button id="crosswalk-stopped" type="button" disabled>已确认车轮停稳</button>
<button id="crosswalk-continue" type="button" disabled>测量完成 · 继续 0.5 秒</button>
<p id="crosswalk-speech-state"></p>
<button id="crosswalk-cancel" type="button">取消停车试验</button>
</section>'''

CONTROL_SCRIPT = '''
el('crosswalk-start').addEventListener('click',async()=>{
 if(!el('crosswalk-field-ready').checked){feedback('先确认场地空旷且现场可断开电调','error');return;}
 held.clear();needsRelease=true;gamepadControl?.requireNeutral();
 try{
  render(await send('stop',{stop_source:'stop_button'}));
  crosswalkActive=true;
  render(await send('crosswalk_start',{field_ready:true,path_mode:el('crosswalk-path-mode').value}));
 }catch(e){crosswalkActive=false;feedback(e.message,'error');}
});
el('crosswalk-stopped').addEventListener('click',async()=>{
 try{render(await send('crosswalk_stopped',{wheels_stopped:true}));}catch(e){feedback(e.message,'error');}
});
el('crosswalk-continue').addEventListener('click',async()=>{
 try{crosswalkActive=true;render(await send('crosswalk_continue'));}
 catch(e){crosswalkActive=false;feedback(e.message,'error');}
});
el('crosswalk-cancel').addEventListener('click',()=>stop('stop',false,'stop_button'));
setInterval(async()=>{
 if(!crosswalkActive||stale||crosswalkPulseBusy>=2)return;
 crosswalkPulseBusy++;
 try{render(await send('crosswalk_keepalive'));}catch(e){crosswalkActive=false;stop('stop',false,'connection_recovery');}
 finally{crosswalkPulseBusy--;}
},100);
'''


def extend_controls(controls):
    script = controls.SCRIPT
    sentinel = 'let gamepadControl=null,pwmControl=null;'
    render = 'function render(s){'
    heartbeat = 'async function heartbeat(){'
    ending = script.rfind('})();')
    if any(script.count(s) != 1 for s in (sentinel, render, heartbeat)) or ending < 0:
        raise ValueError("当前控制台脚本与已审查的 10 月 9 日版本不一致，停止加载扩展")
    script = script.replace(sentinel, sentinel + '\nlet crosswalkActive=false,crosswalkPulseBusy=0;')
    script = script.replace(render, render + '''
 sequence=Math.max(sequence,s.last_command_sequence??-1);
 crosswalkActive=!!s.crosswalk_trial?.active;
 el('crosswalk-test-state').textContent=s.crosswalk_trial?.reason||'等待停车测试服务';
 el('crosswalk-reference-state').textContent=s.crosswalk_trial?.reference_summary||'';
 el('crosswalk-start').disabled=!!crosswalkActive||s.mode!=='enabled'||!s.crosswalk_trial?.reference_available||s.crosswalk_trial?.trial_enabled===false;
 el('crosswalk-stopped').disabled=s.crosswalk_trial?.phase!=='braking';
 el('crosswalk-continue').disabled=!s.crosswalk_trial?.can_continue||s.mode!=='enabled';
 el('crosswalk-speech-state').textContent=({sending:'正在发送播报',command_sent:'已发送：我停车了啊',failed:'播报失败，请检查语音模块',cancelled:'播报已取消'})[s.crosswalk_trial?.speech?.state]||'';
 if(crosswalkActive){held.clear();needsRelease=true;gamepadControl?.requireNeutral();}
''')
    script = script.replace(heartbeat, heartbeat + '\n if(crosswalkActive){heartbeatDue=false;return;}')
    ending = script.rfind('})();')
    controls.SCRIPT = script[:ending] + CONTROL_SCRIPT + script[ending:]
    controls.HTML += CONTROL_HTML


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--reference', type=Path)
    mode.add_argument('--brake-config', type=Path, help='F/B brake program on the existing real-camera console')
    p.add_argument('--traffic-config', type=Path, help='Add supervised red-stop / green-go driving to the existing F/B console')
    p.add_argument('--feedback', type=Path, help='atomic verified speed JSON on this Pi; absent means operator confirms stopped')
    p.add_argument('--port', type=int, default=8080)
    args = p.parse_args(argv)
    if args.traffic_config and not args.brake_config:
        p.error('--traffic-config requires --brake-config (F/B actuator mode)')
    root = args.root.resolve()
    boot_path = root/'vision/tools/serve_boot_test.py'
    if not boot_path.is_file():
        raise ValueError('需要当前车端 serve_boot_test.py；不能用旧仓库启动器替代')
    sys.path.insert(0, str(root/'vision/src'))
    sys.path.insert(0, str(root/'vision/tools'))
    from carvision import manual_controls, web_preview
    from carvision.crosswalk_trial import CrosswalkTrialDrive
    from carvision.parking_speech import ParkingSpeech
    if args.brake_config:
        from carvision.brake_controls import extend_controls as extend_brake_controls
        extend_brake_controls(manual_controls)
        if args.traffic_config:
            from carvision.traffic_controls import extend_controls as extend_traffic_controls
            extend_traffic_controls(manual_controls)
    else:
        extend_controls(manual_controls)
    spec = importlib.util.spec_from_file_location('current_boot_console', boot_path)
    boot = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(boot)
    original_serve = boot.serve
    original_make = web_preview.make_server

    def serve(args_for_camera, config, drive=None):
        if args.brake_config:
            from carvision.brake_trial import BrakeTrialDrive
            wrapper = BrakeTrialDrive(drive, {}, args.brake_config, root)
        else:
            wrapper = CrosswalkTrialDrive(drive, {}, args.reference, root/'run/crosswalk-parking-trial.jsonl',
                                         feedback_path=args.feedback, speech_factory=lambda:ParkingSpeech(root))
        parking = wrapper
        if args.traffic_config:
            from carvision.traffic_trial import TrafficTrialDrive
            wrapper = TrafficTrialDrive(parking, {}, args.traffic_config, root/'vision/configs/traffic-signal.json', root)
        # The parking decision consumes the secondary raw frames. Keep the
        # existing camera owner/format/pose, but avoid the old ~2.5 Hz raw feed.
        args_for_camera.secondary_raw_preview_fps = 15
        def make_server(address, state, secondary=None, drive=None):
            wrapper.states = {'primary':state, **({'secondary':secondary} if secondary else {})}
            parking.states = wrapper.states
            server = original_make(address, state, secondary, wrapper)
            if args.traffic_config:
                from carvision.traffic_controls import add_preview_endpoint
                add_preview_endpoint(server, wrapper)
            return server
        web_preview.make_server = make_server
        try:
            return original_serve(args_for_camera, config, drive=wrapper)
        finally:
            wrapper.close()
            web_preview.make_server = original_make

    boot.serve = serve
    previous_argv = sys.argv
    sys.argv = [str(boot_path), '--root', str(root), '--port', str(args.port)]
    try:
        return boot.main()
    finally:
        sys.argv = previous_argv


if __name__ == '__main__':
    main()
