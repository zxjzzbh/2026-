#!/usr/bin/env python3
"""比赛总入口：默认观察；--drive 后等待蓝板移开，自动运行至停车扫码。"""
import argparse
from datetime import datetime
from copy import deepcopy
import hashlib
import json
import math
import os
import queue
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import zipfile

import cv2
import config as C
from parking_record_camera import RecordingCamera
from cone_solo import restore_gimbal
from fusion_logic import Settings
from fusion_run import FusionIO, runtime_config, play_audio, draw_frame, _make_frame_record
from light_cone_run import load_light_config
from race_master_logic import RaceController, RaceBarrierGate
from lane_follow import LaneFollower
from lane_follow_relaxed import RelaxedLaneFollower
from race_manual import TerminalKeys, ManualControls, ParkingDeadline, manual_allowed, skip_controller


ROOT=Path(__file__).resolve().parent
TRAFFIC_DRIVE_DUTY=12650

from race_loop_diagnostics import LoopDiagnostics, main_thread_stack


class SnapshotWriter:
    """Keep occasional JPEG and filesystem stalls outside the motion loop."""
    def __init__(self, folder):
        self.folder = folder
        self.pending = queue.Queue(maxsize=3)
        self.dropped = 0
        self.errors = 0
        self.max_write_ms = 0.
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def submit(self, fid, stage, raw, debug):
        try:
            self.pending.put_nowait((fid, stage, raw.copy(), debug))
        except queue.Full:
            self.dropped += 1

    def _write(self):
        while True:
            item = self.pending.get()
            if item is None:
                return
            fid, stage, raw, debug = item
            for label, image in (('raw', raw), ('debug', debug)):
                start = time.monotonic()
                try:
                    dest = self.folder / ('%06d_%s_%s.jpg' % (fid, stage, label))
                    if not cv2.imwrite(str(dest), image):
                        self.errors += 1
                except (OSError, cv2.error):
                    self.errors += 1
                self.max_write_ms = max(self.max_write_ms, 1000*(time.monotonic()-start))

    def close(self):
        self.pending.put(None)
        self.worker.join()


def load_config(path):
    cfg=json.loads(Path(path).read_text(encoding='utf8'))
    allowed=[['CROSSWALK','TRAFFIC','CONES','PARK_SEARCH'],
             ['CROSSWALK','CONES','TRAFFIC','PARK_SEARCH'],
             ['CROSSWALK','CONES','PARK_SEARCH']]
    if cfg['stage_order'] not in allowed: raise ValueError('Invalid stage order')
    if cfg['hold_seconds']!=10 or cfg['expected_cones']!=2:
        raise ValueError('Race requires 10 seconds crosswalk hold and two cones')
    if not 0<cfg['qr_seconds']<=20 or not 0<cfg['max_seconds']<=1800:
        raise ValueError('Invalid time limits')
    traffic_duty=cfg.get('traffic_approach_duty',12300)
    if not math.isfinite(traffic_duty) or not C.STOP_DUTY<traffic_duty<=cfg['cruise_duty']:
        raise ValueError('Invalid traffic approach duty')
    roi=cfg['barrier_roi']
    if not (len(roi)==4 and all(math.isfinite(v) for v in roi)
            and 0<=roi[0]<roi[2]<=1 and 0<=roi[1]<roi[3]<=1):
        raise ValueError('Invalid barrier ROI')
    if not 0<=cfg['lane_roi'][0]<cfg['lane_roi'][1]<=1:
        raise ValueError('Invalid lane ROI')
    if not math.isfinite(cfg['parking_settle_seconds']) or cfg['parking_settle_seconds']<.3:
        raise ValueError('Parking stationary settle must be at least .3 seconds')
    return cfg


def build_controller(cfg):
    settings=Settings(cruise_duty=cfg['cruise_duty'],cone_duty=cfg['cone_duty'],
        approach_duty=cfg['approach_duty'],hold_seconds=10,resume=True,
        cone_steer_slew_us_s=1800,brake_on_cone_fault=True,
        cone_protection_enabled=cfg['cone_protection'])
    mode=cfg.get('lane_mode','original')
    if mode not in ('original','relaxed'): raise ValueError('Invalid lane mode')
    lane=RelaxedLaneFollower() if mode=='relaxed' else LaneFollower()
    cal=load_light_config(ROOT/'light_stop_calibration.json')
    return RaceController(settings,cfg,cal['traffic'],lane=lane)


def build_race_controller(cfg):
    from crosswalk_v11.sampled import SampledCrosswalk
    from race_parking_current import install_new_parking
    ctl=build_controller(cfg)
    if 'CROSSWALK' in cfg['stage_order']:
        ctl.cross = SampledCrosswalk()
    if ('TRAFFIC' in cfg['stage_order'] and cfg.get('traffic_mode','green')=='green'
            and not cfg.get('backup_light_detector')):
        # Use the sibling package explicitly: both projects have a config.py and motor modules.
        src=str(Path(__file__).resolve().parents[1])
        if src not in sys.path: sys.path.append(src)
        from light_straight_20261114 import light_straight
        from race_traffic_board import install
        install(ctl, light_straight.make_detector(), TRAFFIC_DRIVE_DUTY)
    return install_new_parking(ctl)


def check_dependencies(cfg):
    for key in ('announcement_file','qr_script'):
        if not Path(cfg[key]).is_file(): raise ValueError('Missing '+key+': '+cfg[key])
    build_race_controller(cfg)
    import importlib.util
    if importlib.util.find_spec('pigpio') is None: raise ValueError('pigpio Python module missing')


def check_conflicts():
    from recorder_parent_guard import is_own_recorder
    names={'fusion_field_run.py','fusion_run.py','competition_run.py','cone_solo.py',
           'light_cone_run.py','light_stop_test.py','parking_side_exit_run.py',
           'parking_bypass_trial.py','parking_qr_run.py','race_master_run.py'}
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name)==os.getpid(): continue
        try: parts=(proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
        except OSError: continue
        if not parts: continue
        if is_own_recorder(parts,int(proc.name),os.getppid(),__file__):continue
        if (('python' in Path(parts[0]).name and any(Path(a).name in names for a in parts[1:]))
                or any(a.endswith('/lib/gpio.js') for a in parts)):
            raise RuntimeError('Other vehicle controller active: PID '+proc.name)
        # Match the executable script, so a recorder's --script argument is harmless.
        if 'python' in Path(parts[0]).name:
            script = next((arg for arg in parts[1:] if not arg.startswith('-')), '')
            if Path(script).name in {'full_race_light_test.py','light_straight.py','light_lane_run.py','cone_lane_run.py'}:
                raise RuntimeError('Other vehicle controller active: PID '+proc.name)


def close_motion(car, camera):
    try:
        if car is not None: car.close()
    finally:
        if camera is not None: camera.stop()


def capture_payment(cfg, name):
    if name.startswith('web_'):
        worker = ROOT / 'remote_console' / 'idle_qr.py'
        photo = ROOT / 'runs' / name / 'parked_front.jpg'
        result = subprocess.run([sys.executable, '-I', str(worker), '--after-stop'],
                                stdout=subprocess.PIPE, check=True, timeout=12)
        if not (result.stdout.startswith(bytes.fromhex('ffd8')) and
                result.stdout.endswith(bytes.fromhex('ffd9'))):
            raise ValueError('Incomplete front camera JPEG')
        temp = photo.with_suffix('.tmp')
        temp.write_bytes(result.stdout)
        temp.replace(photo)
        return dict(status='PHOTO_READY', image=str(photo), payment_completed=False)
    script=Path(cfg['qr_script'])
    result=subprocess.run([sys.executable,'-u',str(script),'--seconds',str(cfg['qr_seconds']),
                           '--name',name+'_payment'],cwd=str(script.parent),timeout=cfg['qr_seconds']+8)
    ready=script.parent/'captures'/(name+'_payment')/'qr_payment_scan_ready.png'
    return dict(status='QR_READY' if result.returncode==0 and ready.is_file() else 'QR_FAILED',
                image=str(ready) if ready.is_file() else None, payment_completed=False)


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--drive',action='store_true')
    p.add_argument('--check',action='store_true',help='检查配置和依赖，不打开硬件')
    p.add_argument('--self-test',action='store_true',help='离线状态机测试，不打开硬件')
    p.add_argument('--config',type=Path,default=ROOT/'race_master_config.json')
    p.add_argument('--order',choices=('rules','cones-first'))
    p.add_argument('--color',choices=('red','blue'))
    p.add_argument('--lane-mode',choices=('original','relaxed'),default='original',
                   help='original: current lane checks; relaxed: optional wider geometry tolerances')
    p.add_argument('--traffic-mode',choices=('green','red-timer'),default='green',
                   help='green: release on red off or green; red-timer: stop on red, wait 5.5s, then continue')
    p.add_argument('--cone-duty',type=int)
    p.add_argument('--cone-mode',choices=('joint','original','relaxed'),default='joint',
                   help='joint: tuned cones with parking lane following; original/relaxed: legacy controllers')
    p.add_argument('--seconds',type=float)
    p.add_argument('--name',default=None)
    return p.parse_args(argv)


def run(args,cfg,*,parking_only=False,traffic_only=False):
    from race_parking_current import PARKING_STAGES, sync_parking_age, steering_snapshot, parking_driver_command
    from parking_only_test import LeftLineCapture, YellowLineCapture
    check_conflicts()
    import fcntl
    lock=open('/tmp/wisecar_race_master.lock','a')
    try: fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except OSError:
        lock.close()
        raise RuntimeError('Race master already running')
    prefix='traffic_test_' if traffic_only else 'parking_' if parking_only else 'race_'
    name=args.name or datetime.now().strftime(prefix+'%Y%m%d_%H%M%S_%f')
    if Path(name).name!=name or name in ('.','..') or any(c in name for c in '\\:'):
        lock.close()
        raise ValueError('Invalid run name')
    folder=ROOT/'runs'/name
    folder.mkdir(parents=True,exist_ok=False)
    ctl=build_race_controller(cfg)
    signal_controller=getattr(ctl.policy,'signal',None)
    if signal_controller is not None:
        cfg=dict(cfg,traffic_approach_duty=signal_controller.duty)
    if traffic_only:
        if signal_controller is None or ctl.policy.stage != 'TRAFFIC':
            raise ValueError('Expected the full-race traffic-light stage')
        ctl.policy.cross_done=True
        ctl.policy.transition_reason='START_TRAFFIC_ONLY'
    if parking_only:
        ctl.policy.stage='PARK_SEARCH'
        ctl.policy.cross_done=True
        ctl.policy.transition_reason='START_PARKING_ONLY'
    manual=ManualControls(clock=time.monotonic)
    ctl.policy.clock=manual.clock
    parking_deadline=ParkingDeadline()
    keyboard=TerminalKeys()
    skipped_modules=[]
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*.py')}
    (folder/'run.json').write_text(json.dumps(dict(config=cfg,drive=args.drive,
        entry_mode='traffic-only' if traffic_only else 'parking-only' if parking_only else 'full-race',
        lane_detector='WHITE_TAPE_V3_OBSERVED_'+cfg.get('lane_mode','original').upper(),
        source_hashes=hashes),ensure_ascii=False,indent=2),encoding='utf8')
    stages=(['PARK_SEARCH','PARK_SETTLE','PARKING','QR_CAPTURE'] if parking_only
            else ['WAIT_BARRIER']+cfg['stage_order']+['PARKING','QR_CAPTURE'])
    if traffic_only:
        stages=cfg['stage_order']
    print('[race] '+' -> '.join(stages),flush=True)
    drive_mode='DRIVE: parking-only' if parking_only else 'DRIVE: wait for blue board removal'
    if traffic_only:
        drive_mode='DRIVE: traffic-light stage only'
    print('[mode] '+(drive_mode if args.drive else 'OBSERVE: no motor/gimbal GPIO'),flush=True)
    print('[logs]',folder,flush=True)
    print('[traffic mode]',cfg.get('traffic_mode','green'),
          'stop on red, release once; duty %d'%signal_controller.duty if signal_controller else '',flush=True)
    print('[lane mode]',cfg.get('lane_mode','original'),flush=True)
    if not traffic_only:
        print('[cone mode]','joint' if cfg.get('joint_cones') else cfg.get('cone_mode','original'),flush=True)
        print('[parking] blue board -> outer white -> entry yellow (90% / near-bottom loss 85%) -> right 0.6s / left 0.6s -> stop',flush=True)
    camera=car=None
    previous_signals={}
    done=threading.Event()
    guard_thread=None
    last_capture=[None]
    loop_diagnostics=LoopDiagnostics()
    watchdog_fault=[None]
    control_thread_id=threading.get_ident()
    status='STARTING'
    frames=0
    parking_failures={}
    left_loss=LeftLineCapture()
    yellow_loss=None
    qr=None
    snapshots = SnapshotWriter(folder)
    joint_cones=None
    def interrupted(signum, _): raise KeyboardInterrupt
    try:
        if cfg.get('joint_cones') and not parking_only:
            from race_joint_cones import JointCones
            joint_cones=JointCones(cfg)
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            previous_signals[sig]=signal.signal(sig,interrupted)
        keyboard.__enter__()
        print('[keys] S pause/resume; D skip current task; Ctrl+C exit' if keyboard.enabled
              else '[keys] unavailable: use a foreground SSH terminal (ssh -t)',flush=True)
        if args.drive:
            car=FusionIO(C)
            if 'CROSSWALK' not in cfg['stage_order'] and not traffic_only:
                restore_gimbal(C)
            else:
                print('[camera] Keeping current sampled pose; no gimbal reset',flush=True)
        camera=RecordingCamera(folder)
        camera.start()
        begin=time.monotonic()
        def deadlines():
            while not done.wait(.01):
                now=time.monotonic()
                if now-begin>=cfg['max_seconds']:
                    car.stop('RACE_TIME_LIMIT'); return
                if not car.enabled: continue
                # In the full race, slow logging after an applied command does not age
                # the camera stream. Keep the independent guard on actual capture loss.
                captured_at = last_capture[0]
                if not (parking_only or traffic_only) and C.MAX_FRAME_AGE_SECONDS == 0.20:
                    captured_at = camera.timestamp
                # Parking checks each new frame before applying its command.
                # Time spent writing the previous frame's log is not a parking fault.
                if (ctl.policy.stage not in PARKING_STAGES + ('QR_CAPTURE',)
                        and captured_at is not None and now - captured_at > C.MAX_FRAME_AGE_SECONDS):
                    fault=loop_diagnostics.snapshot(now)
                    fault.update(reason='HARD_FRAME_AGE_LIMIT',
                                 limit_ms=C.MAX_FRAME_AGE_SECONDS*1000.,
                                 camera_latest_age_ms=(None if getattr(camera,'timestamp',None) is None
                                                       else (now-camera.timestamp)*1000.))
                    car.stop('HARD_FRAME_AGE_LIMIT')
                    fault['main_thread_stack']=main_thread_stack(control_thread_id)
                    watchdog_fault[0]=fault
                    return
                if parking_deadline.expired(now):
                    car.stop('HARD_PARK_PASS_LIMIT'); return
        if car is not None:
            guard_thread=threading.Thread(target=deadlines,daemon=True)
            guard_thread.start()
        gate=None if parking_only or traffic_only else RaceBarrierGate(cfg['barrier_roi'])
        clear_at=None
        last_id=None
        previous_key=None
        last_print=last_image=0.
        with (folder/'events.jsonl').open('w',encoding='utf8',buffering=1) as log:
            while time.monotonic()-begin<cfg['max_seconds']:
                sync_parking_age(ctl)
                if joint_cones and ctl.policy.stage=='CONES':
                    C.MAX_FRAME_AGE_SECONDS=joint_cones.max_age
                loop_diagnostics.begin_frame()
                frame,fid,ts=camera.read()
                loop_diagnostics.frame(fid,ts)
                loop_diagnostics.enter('frame_and_keyboard_checks')
                now=time.monotonic()
                if car is not None and car.reason:
                    status=car.reason; break
                if frame is None or not math.isfinite(ts) or not 0<=now-ts<=C.MAX_FRAME_AGE_SECONDS:
                    status='STALE_FRAME'; break
                last_capture[0]=ts
                keypress=keyboard.poll()
                if keypress:
                    if manual_allowed(ctl,car):
                        if manual.request(keypress,now,moving=(car is not None and car.enabled and car.current>C.STOP_DUTY)):
                            action=('PAUSED' if manual.paused else 'RESUME_REQUESTED') if keypress=='s' else 'SKIP_REQUESTED'
                            print('[manual]',action,flush=True)
                            log.write(json.dumps(dict(t=now-begin,manual=action,
                                mission_stage='WAIT_BARRIER' if gate is not None else ctl.policy.stage))+'\n')
                    else:
                        print('[manual] ignored: fault stop remains active',flush=True)
                holding=manual.update(now)
                if holding:
                    # Keep fresh neutral/brake commands flowing to the real-time
                    # watchdog, without changing the policy's saved task state.
                    if car is not None and car.enabled: car.write_command(manual.stop_command(now),ts)
                    # Keep the old real parking guard until a safe stop command
                    # has actually been applied, including a slow/failed write.
                    parking_deadline.suspend()
                    if manual.ready_to_skip(now):
                        old_stage='WAIT_BARRIER' if gate is not None else ctl.policy.stage
                        if gate is not None:
                            gate=None
                            clear_at=None
                            target=ctl.policy.stage
                        elif traffic_only:
                            target='MANUAL_FINISHED'
                        else:
                            ctl,target=skip_controller(ctl,lambda:build_race_controller(cfg),manual.clock())
                            ctl.policy.clock=manual.clock
                        skipped_modules.append('PARKING' if old_stage in ('PARK_SEARCH','PARK_SETTLE','PARKING') else old_stage)
                        print('[manual] SKIP %s -> %s%s'%(old_stage,target,' (paused)' if manual.paused else ''),flush=True)
                        log.write(json.dumps(dict(t=now-begin,manual='SKIPPED',module=old_stage,next_stage=target,
                            paused=manual.paused))+'\n')
                        manual.complete_skip(now)
                        previous_key=None
                        if target=='MANUAL_FINISHED':
                            status='MANUAL_FINISHED'
                            if car is not None: car.stop(status)
                            break
                    time.sleep(.002)
                    continue
                # Consume a frame captured after resume/skip; queued old frames
                # must never restart motion or move backwards on the mission clock.
                if manual.fresh_after is not None and ts<=manual.fresh_after:
                    if car is not None and car.enabled: car.write_command(manual.stop_command(now),ts)
                    time.sleep(.002)
                    continue
                if fid==last_id:
                    time.sleep(.002); continue
                last_id=fid
                mission_now=manual.clock.translate(now)
                mission_ts=manual.clock.translate(ts)
                if gate is not None:
                    gr=gate.update(frame)
                    event=dict(frame=fid,t=now-begin,mission_stage='WAIT_BARRIER',gate=gr.state,
                               blue_ratio=gr.blue_ratio,actual_duty=C.STOP_DUTY)
                    if gr.go: clear_at=mission_now+C.BARRIER_CLEAR_SETTLE_SECONDS
                    if clear_at is not None and mission_now>=clear_at:
                        gate=None
                        print('[stage] WAIT_BARRIER -> CROSSWALK',flush=True)
                    log.write(json.dumps(event)+'\n')
                    if now-last_print>=.5:
                        print('[barrier] %s blue=%.3f'%(gr.state,gr.blue_ratio),flush=True)
                        cv2.imwrite(str(folder/'latest.jpg'),frame)
                        last_print=now
                    continue
                before_cross=ctl.cross_result
                observation_stage=ctl.policy.stage
                if observation_stage in PARKING_STAGES:
                    if yellow_loss is None:
                        yellow_loss=YellowLineCapture(folder)
                    yellow_loss.begin_frame()
                joint_event=None
                loop_diagnostics.enter('perception_and_control')
                if joint_cones and observation_stage=='CONES':
                    cmd,joint_event=joint_cones.step(ctl,frame,fid,mission_ts,mission_now)
                else:
                    cmd=ctl.step(frame,fid,mission_ts,mission_now)
                if cmd is None: continue
                if time.monotonic()-ts>C.MAX_FRAME_AGE_SECONDS:
                    cmd=ctl.policy.halt('PROCESSING_TOO_SLOW')
                elif traffic_only and ctl.policy.stage=='TRAFFIC_COMPLETE':
                    # The normal controller has released the light and requested neutral.
                    cmd.state='DONE'
                driver_cmd=parking_driver_command(ctl,cmd)
                loop_diagnostics.enter('motor_write')
                if car is not None:
                    if not car.enabled and driver_cmd.duty>C.STOP_DUTY: car.arm()
                    if car.enabled or driver_cmd.state in ('HALT','DONE'): car.write_command(driver_cmd,ts)
                    if car.reason=='PARKED_QR_READY' and name.startswith('web_'):
                        stamp=folder/'parked.tmp'
                        stamp.write_text(json.dumps({'run':name,'at_ms':int(time.time()*1000)}),encoding='utf8')
                        stamp.replace(folder/'parked.json')
                    parking_deadline.observe(ctl.policy,manual.clock(),time.monotonic(),
                        forward=car.enabled and driver_cmd.duty>C.STOP_DUTY)
                loop_diagnostics.enter('post_control')
                if observation_stage in PARKING_STAGES:
                    yellow_loss.collect(frame,fid,now-begin,observation_stage,ctl.parking_observation or {})
                    if 'ordinary_lane_ms' in ctl.step_timing:
                        left_loss.add(frame,ctl.lane_result,fid,now-begin,observation_stage)
                key=(ctl.policy.stage,cmd.state)
                if key!=previous_key: ctl.policy.mark_applied(cmd,manual.clock())
                if cmd.announce and args.drive: play_audio(cfg['announcement_file'])
                if joint_event is None:
                    event=_make_frame_record(ctl,cmd,fid,now,ts,begin,None,car,before_cross)
                else:
                    event=joint_event
                    event.update(frame=fid,t=now-begin,mission_stage=ctl.policy.stage,
                                 state=cmd.state,owner=cmd.owner,reason=cmd.reason,duty=cmd.duty,
                                 steer_us=cmd.steer_us,actual_duty=car.current if car else None)
                event.update(mission_t=mission_now-begin,manual_pause_seconds=manual.clock.offset)
                skipped=getattr(ctl,'perception_skipped',None) if joint_event is None else None
                event['perception_skipped']=skipped
                if skipped:
                    event.update(lane_valid=None,lane_mode='NOT_EVALUATED_HOLD')
                if joint_event is None and ctl.policy.stage in ('PARK_SEARCH','PARK_SETTLE','PARKING','QR_CAPTURE'):
                    event['steering'] = steering_snapshot(ctl,cmd,driver_cmd,car,mission_ts)
                    guide=(ctl.parking_observation or {}).get('lane_guide') or {}
                    reason=(cmd.reason if cmd.reason.startswith('ENTRY_YELLOW_')
                            else guide.get('reason') if not guide.get('valid') else None)
                    if (reason and reason not in ('OUTER_WHITE_CONFIRMING','NO_CONFIRMED_CORRIDOR')
                            and reason not in parking_failures and len(parking_failures)<12):
                        parking_failures[reason]=(frame.copy(),deepcopy(event))
                loop_diagnostics.enter('event_log')
                event['loop_timing_ms']=loop_diagnostics.snapshot()['timing_ms']
                log.write(json.dumps(event,ensure_ascii=False)+'\n')
                frames+=1
                loop_diagnostics.enter('debug_render')
                if key!=previous_key or now-last_image>=1:
                    debug=frame.copy() if joint_event is not None or skipped else draw_frame(frame,ctl,cmd)
                    if joint_event is not None:
                        for box in joint_event.get('boxes', []):
                            x,y,w,h=(box[k] for k in ('x','y','w','h'))
                            cv2.rectangle(debug,(x,y),(x+w-1,y+h-1),(0,255,0),2)
                    loop_diagnostics.enter('snapshot_enqueue')
                    snapshots.submit(fid, ctl.policy.stage, frame, debug)
                    last_image=now
                loop_diagnostics.enter('terminal_print')
                if key!=previous_key or now-last_print>=.5:
                    print('[stage] %s %s owner=%s duty=%s reason=%s cones=%d/%d'%(
                        ctl.policy.stage,cmd.state,cmd.owner,cmd.duty,cmd.reason,
                        len(ctl.policy.completed_cones),cfg['expected_cones']),flush=True)
                    last_print=now
                previous_key=key
                loop_diagnostics.enter('loop_tail')
                if cmd.state in ('HALT','DONE'):
                    status=(car.reason if car is not None and car.reason else cmd.reason)
                    break
            else: status='RACE_TIME_LIMIT'
    except KeyboardInterrupt:
        status='INTERRUPTED'
    except Exception as exc:
        status='ERROR: '+str(exc)
        print(status,flush=True)
    finally:
        done.set()
        if guard_thread is not None: guard_thread.join(timeout=.3)
        try: close_motion(car,camera)
        finally:
            snapshots.close()
            if joint_cones is not None: joint_cones.close()
            try: keyboard.close()
            finally:
                for sig,previous in previous_signals.items(): signal.signal(sig,previous)
    for index,(reason,(raw,event)) in enumerate(parking_failures.items()):
        stem=folder/('parking_failure_%02d_%s'%(index,reason))
        cv2.imwrite(str(stem)+'.jpg',raw)
        Path(str(stem)+'.json').write_text(json.dumps(event,ensure_ascii=False,indent=2),encoding='utf8')
    # Only the normal stopped endpoint may open the independent high-res camera.
    try:
        if status=='PARKED_QR_READY' and car is not None and car.reason!='PARKED_QR_READY':
            status=car.reason or 'DRIVER_STOP_NOT_CONFIRMED'
        if status=='PARKED_QR_READY' and args.drive:
            print('[qr] motor and motion camera closed; capturing payment image',flush=True)
            try: qr=capture_payment(cfg,name)
            except subprocess.TimeoutExpired: qr=dict(status='QR_TIMEOUT',payment_completed=False)
            except Exception as exc: qr=dict(status='QR_ERROR',error=str(exc),payment_completed=False)
            print('[qr]',json.dumps(qr,ensure_ascii=False),flush=True)
        summary=dict(status=status,frames=frames,stage=ctl.policy.stage,
            skipped_modules=skipped_modules,manually_skipped=bool(skipped_modules),
            confirmed_cones=len(ctl.policy.completed_cones),
            expected_cones=cfg['expected_cones'],
            all_cones_confirmed=len(ctl.policy.completed_cones)>=cfg['expected_cones'],
            parking_verified=False,qr=qr,payment_completed=False)
        summary['snapshots'] = dict(dropped=snapshots.dropped, errors=snapshots.errors,
                                    max_write_ms=round(snapshots.max_write_ms, 1))
        summary['loop_max_ms']={k:round(v,3) for k,v in loop_diagnostics.max_ms.items()}
        summary['watchdog_fault']=watchdog_fault[0]
        if yellow_loss is not None:
            summary['yellow_line_loss']=yellow_loss.finish()
        summary['left_loss_clips']=len(left_loss.clips)
        summary['left_loss_skipped']=left_loss.skipped
        try:
            left_loss.save(folder)
        except (OSError, cv2.error) as exc:
            summary['left_loss_save_error']=str(exc)
        (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf8')
        try:
            from race_replay import export
            print('[replay]', export(folder), flush=True)
        except Exception as exc:
            print('[replay error]', exc, flush=True)
        with zipfile.ZipFile(str(folder)+'.zip','w',zipfile.ZIP_DEFLATED) as archive:
            for p in folder.rglob('*'):
                if p.is_file(): archive.write(str(p),str(Path(name)/p.relative_to(folder)))
        print('[result]',json.dumps(summary,ensure_ascii=False),flush=True)
    finally: lock.close()
    if traffic_only:
        return 0 if status in ('GREEN_CONFIRMED','RED_OFF_CONFIRMED') else 1
    return 0 if status=='PARKED_QR_READY' and (not args.drive or qr and qr['status'] in ('PHOTO_READY','QR_READY')) else 1


def main(argv=None):
    args=parse_args(argv)
    if args.self_test:
        import unittest
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/'tests')))
        return 0 if result.wasSuccessful() else 1
    cfg=load_config(args.config)
    cfg['traffic_mode']=args.traffic_mode
    cfg['lane_mode']=args.lane_mode
    cfg['joint_cones']=args.cone_mode=='joint'
    cfg['cone_mode']='original' if cfg['joint_cones'] else args.cone_mode
    if args.order: cfg['stage_order']=['CROSSWALK']+(['TRAFFIC','CONES'] if args.order=='rules' else ['CONES','TRAFFIC'])+['PARK_SEARCH']
    # Temporary joint test: also skip traffic when an order override is supplied.
    if cfg.get('skip_traffic', False):
        cfg['stage_order']=[stage for stage in cfg['stage_order'] if stage!='TRAFFIC']
    if args.color: cfg['cone_color']=args.color
    elif cfg['joint_cones']: cfg['cone_color']='blue'
    if args.cone_duty is not None: cfg['cone_duty']=args.cone_duty
    if args.seconds is not None:
        if not math.isfinite(args.seconds) or not 0<args.seconds<=1800: raise ValueError('seconds must be in (0,1800]')
        cfg['max_seconds']=args.seconds
    cv2.setNumThreads(1)
    with runtime_config(True,cfg['lane_roi']):
        check_dependencies(cfg)
        if args.check:
            check_conflicts()
            if cfg['joint_cones']:
                from race_joint_cones import JointCones
                cones=JointCones(cfg)
                try:
                    print('[joint cones]',cones.settings['cone_mode'],cfg['cone_color'],
                          cones.settings['lane_version'],cones.settings['expected_cones'])
                finally: cones.close()
            print('[check] OK: modules, camera calibration, audio, QR script; no hardware opened')
            print('[order]', ' -> '.join(cfg['stage_order']))
            if 'CROSSWALK' in cfg['stage_order']:
                print('[crosswalk] V11 / sampled 120cm image reference / duty %s / brake 10600 for 0.25s / 10s hold / keep current camera pose'%cfg.get('crosswalk_drive_duty'))
            print('[traffic mode]',cfg['traffic_mode'])
            print('[lane mode]',cfg['lane_mode'])
            print('[cone mode]',args.cone_mode)
            print('[parking] blue board -> outer white -> entry yellow (90% / near-bottom loss 85%) -> right 0.6s / left 0.6s -> stop')
            return 0
        return run(args,cfg)


if __name__=='__main__': raise SystemExit(main())
