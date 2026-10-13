"""Install departure, parking observation and payment around the idle console.

Preserves installed cone/traffic/parking/manual files and service arguments.
Never scans a code, starts a payment window, drives, or transfers money.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import shlex
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import paramiko
from deploy_brake_trial import command,put_atomic,read,ROOT,REMOTE,PYTHON,UNIT
from deploy_traffic_trial import PROTECTED as BASE_PROTECTED, idle as base_idle

sys.path.insert(0,str(ROOT/'vision/src'))
from carvision.brake_controls import extend_controls as brake
from carvision.traffic_controls import extend_controls as traffic
from carvision.payment_controls import extend_controls as payment
from carvision.departure_controls import extend_controls as departure
from carvision.parking_bay_controls import extend_controls as parking_bay

FILES=['vision/src/carvision/'+n+'.py' for n in ('parking_payment','payment_console','payment_controls','departure','departure_trial','departure_controls',
    'parking_bay','parking_mission','parking_bay_console','parking_bay_controls','parking_entry','parking_entry_run','parking_log','parking_guidance','parking_path','parking_control','parking_feedback','parking_far_tracker')]+[
    'vision/tools/serve_parking_payment.py','vision/tests/test_parking_payment.py','vision/tests/test_departure.py',
    'vision/configs/parking-bay.json','vision/tests/test_parking_bay.py','vision/tests/test_parking_path.py','vision/tests/test_parking_entry_run.py','vision/tools/parking_bay.py']
PROTECTED=list(dict.fromkeys(BASE_PROTECTED+[
    'vision/src/carvision/'+n+'.py' for n in ('traffic_driving','traffic_trial','traffic_controls','traffic_signal',
        'traffic_voting','brake_output','cone_vision','cone_road','cone_planning','cone_navigation',
        'cone_execution','cone_observer','cone_controls','cone_short_trial','cone_short_output')]+[
    'vision/tools/serve_crosswalk_trial.py','vision/configs/traffic-driving.json','vision/configs/traffic-signal.json',
    'vision/configs/cone-trial.json','vision/configs/cone-trial.local.json','vision/configs/departure.local.json','vision/configs/parking-bay.local.json']))


def idle(s):
    return (base_idle(s) and not (((s.get('cone_trial') or {}).get('avoidance') or {}).get('active'))
            and not (((s.get('cone_trial') or {}).get('short_test') or {}).get('active'))
            and not (s.get('departure_trial') or {}).get('active')
            and not (s.get('parking_trial') or {}).get('active')
            and not (s.get('race_flow') or {}).get('active')
            and not (s.get('parking_payment') or {}).get('motion_locked'))


def extended_unit(data):
    lines=data.decode('utf-8').splitlines();found=0
    for i,line in enumerate(lines):
        if not line.startswith('ExecStart=') or line=='ExecStart=':continue
        argv=shlex.split(line[len('ExecStart='):])
        if (len(argv)<2 or argv[0]!=PYTHON or argv[1] not in
                (REMOTE+'/vision/tools/serve_crosswalk_trial.py',REMOTE+'/vision/tools/serve_parking_payment.py')
                or '--traffic-config' not in argv or '--brake-config' not in argv):
            raise ValueError('Unknown live launcher; no service command overwritten')
        argv[1]=REMOTE+'/vision/tools/serve_parking_payment.py'
        lines[i]='ExecStart='+shlex.join(argv);found+=1
    if found!=1:raise ValueError('Expected exactly one service command')
    return ('\n'.join(lines)+'\n').encode()


def qr_json_recovery_evidence(c,sftp,password):
    """Narrow recovery for the v1 NumPy bbox serialization fault, not idle data.

    Requires this exact fault in the current process, no active payment record,
    no output subprocess, and no process with the actuator UART open.
    """
    pid=command(c,['systemctl','show','smartcar-preview.service','-p','MainPID','--value']).strip()
    if not pid.isdigit() or int(pid)<=0:raise RuntimeError('No current preview process')
    logs=command(c,['journalctl','-u','smartcar-preview.service','_PID='+pid,'-n','60','--no-pager'])
    if 'TypeError: Object of type int64 is not JSON serializable' not in logs:
        raise RuntimeError('Status failure is not the verified QR serialization defect')
    group=command(c,['systemctl','show','smartcar-preview.service','-p','ControlGroup','--value']).strip()
    if group!='/system.slice/smartcar-preview.service':raise RuntimeError('Unexpected service group')
    members=read(sftp,'/sys/fs/cgroup'+group+'/cgroup.procs').decode().split()
    if set(members)!={pid}:raise RuntimeError('Service has child/output processes; recovery not permitted')
    try:
        record=json.loads(read(sftp,REMOTE+'/run/parking-payment-current.json'))
        if record.get('payment',{}).get('motion_locked'):raise RuntimeError('Payment record still active')
    except FileNotFoundError:pass
    probe="""import glob,os,json
owners=[]
for path in glob.glob('/proc/[0-9]*/fd/*'):
 try:
  if os.readlink(path) in ('/dev/ttyAMA0','/dev/serial0'):owners.append(path.split('/')[2])
 except FileNotFoundError:pass
print(json.dumps(owners))
"""
    owners=json.loads(command(c,['python3','-c',probe],password=password))
    if owners:raise RuntimeError('Actuator UART in use; recovery not permitted')
    return {'fault':'int64_json','pid':pid,'status_unavailable':True,
            'actuator_uart_owners':owners,'output_subprocesses':0,'active_payment_record':False}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--handoff',type=Path,required=True)
    p.add_argument('--recover-qr-json-error',action='store_true',help='Recover the verified v1 int64 JSON defect after UART/process checks')
    args=p.parse_args()
    cfg=json.loads((args.handoff/'connection.json').read_text(encoding='utf-8-sig'))['ssh']
    record=ROOT/'run'/('payment-console-deploy-'+time.strftime('%Y%m%d-%H%M%S'));record.mkdir(parents=True,exist_ok=False)
    audit={'deployed':False,'vehicle_commands_sent':False,'payment_requests_sent':False,'money_transferred':False}
    c=paramiko.SSHClient();c.load_host_keys(str(args.handoff/'attachments/known_hosts.smartcar'))
    c.set_missing_host_key_policy(paramiko.RejectPolicy())
    changed=[];backup={};unit_changed=restarted=False
    def status():return json.loads(command(c,['curl','--fail','--silent','--max-time','3','http://127.0.0.1:8080/api/drive/status']))
    def require_idle():
        try:s=status()
        except Exception:
            if not args.recover_qr_json_error:raise
            audit['recovery_checks']=qr_json_recovery_evidence(c,sftp,cfg['password'])
            return
        if not idle(s):raise RuntimeError('Driving/payment session active; no update performed')
    try:
        c.connect(cfg['host'],port=cfg.get('port',22),username=cfg['username'],password=cfg['password'],look_for_keys=False,allow_agent=False,timeout=6)
        sftp=c.open_sftp()
        require_idle()
        for rel,marker in [('vision/src/carvision/cone_trial_output.py',b'parking_forward_us'),
                           ('vision/src/carvision/cone_road.py',b'thorough_boundary_search'),
                           ('vision/src/carvision/cone_boundaries.py',b'complete_small_sets')]:
            if marker not in read(sftp,REMOTE+'/'+rel):
                raise RuntimeError('路径停车需要同步输出与白线识别依赖；请使用 update-race-flow.cmd 完整更新，未写入文件')
        protected={}
        for rel in PROTECTED:
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
            if protected[rel] is not None:
                dst=record/'protected'/rel;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(protected[rel])
        before_unit=read(sftp,UNIT);new_unit=extended_unit(before_unit)
        tree=ast.parse(protected['vision/src/carvision/manual_controls.py'].decode('utf-8'))
        node=next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SCRIPT' for t in n.targets))
        while isinstance(node,ast.Call):node=node.func.value
        controls=SimpleNamespace(SCRIPT=ast.literal_eval(node),HTML='')
        brake(controls);traffic(controls)
        if '--cone-config' in before_unit.decode():
            # Local cone work may be ahead of the installed version. Compose
            # the exact backed-up project module; never deploy local cone edits.
            live_cone=record/'protected/vision/src/carvision/cone_controls.py'
            spec=importlib.util.spec_from_file_location('installed_cone_payment_check',live_cone)
            cone=importlib.util.module_from_spec(spec);spec.loader.exec_module(cone)
            cone.extend_controls(controls)
        payment(controls)
        departure(controls)
        parking_bay(controls)
        (record/'extended-controls.js').write_text(controls.SCRIPT.strip().removeprefix('<script>').removesuffix('</script>'),encoding='utf-8')
        (record/'service.before.conf').write_bytes(before_unit)
        remote_backup=REMOTE+'/run/'+record.name;command(c,['mkdir','-p',remote_backup])
        put_atomic(sftp,remote_backup+'/service.before.conf',before_unit)
        for rel in FILES:
            try:backup[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:backup[rel]=None
            if backup[rel] is not None:
                dst=record/'before'/rel;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(backup[rel])
                command(c,['mkdir','-p',remote_backup+'/before/'+str(Path(rel).parent).replace('\\','/')])
                put_atomic(sftp,remote_backup+'/before/'+rel,backup[rel])
        for rel in FILES:
            put_atomic(sftp,REMOTE+'/'+rel,(ROOT/rel).read_bytes());changed.append(rel)
        tests=['test_parking_bay.py','test_departure.py','test_parking_payment.py','test_traffic_trial.py','test_traffic_driving.py','test_traffic_voting.py',
               'test_brake_trial.py','test_brake_parking_program.py']
        if '--cone-config' in before_unit.decode():tests+=['test_cone_trial.py','test_cone_navigation.py']
        audit['pi_tests']=command(c,['env','PYTHONPATH='+REMOTE+'/vision/src',PYTHON,'-m','pytest',
            *[REMOTE+'/vision/tests/'+t for t in tests],'-q'],timeout=60)
        require_idle()
        for rel,b in protected.items():
            try:actual=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:actual=None
            if b!=actual:raise RuntimeError('Another update changed '+rel+'; do not restart it')
        put_atomic(sftp,remote_backup+'/service.new.conf',new_unit)
        require_idle()
        unit_changed=True
        command(c,['install','-m','644',remote_backup+'/service.new.conf',UNIT],password=cfg['password'])
        command(c,['systemctl','daemon-reload'],password=cfg['password'])
        restarted=True;command(c,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
        deadline=time.monotonic()+20
        while True:
            try:
                after=status()
                if (after.get('parking_payment') or {}).get('program')!='parking-payment-v1':raise RuntimeError('Payment module not loaded')
                if (after.get('departure_trial') or {}).get('program')!='blue-board-speech-v1':raise RuntimeError('Departure module not loaded')
                if (after.get('parking_bay') or {}).get('program')!='two-bay-parking-v1':raise RuntimeError('Parking bay module not loaded')
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.5)
        if not idle(after):raise RuntimeError('Updated service did not stay unprepared')
        audit.update(deployed=True,backup=remote_backup,preserved_existing_control_files=True,
            cone_preserved='--cone-config' in new_unit.decode(),
            after={k:after.get(k) for k in ('mode','hardware_output','worker_started')},
            payment_phase=after['parking_payment']['phase'],
            sha256={rel:hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in FILES})
    except Exception as exc:
        audit['error']=str(exc)
        if changed:
            try:
                for rel in reversed(changed):
                    if backup[rel] is None:sftp.remove(REMOTE+'/'+rel)
                    else:put_atomic(sftp,REMOTE+'/'+rel,backup[rel])
                if unit_changed:
                    command(c,['install','-m','644',remote_backup+'/service.before.conf',UNIT],password=cfg['password'])
                    command(c,['systemctl','daemon-reload'],password=cfg['password'])
                if restarted:command(c,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
                audit['rolled_back']=True
            except Exception as rollback:audit['rollback_error']=str(rollback)
    finally:
        c.close();(record/'deployment.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(audit,ensure_ascii=False,indent=2))
    return 0 if audit['deployed'] else 1


if __name__=='__main__':raise SystemExit(main())
