"""Deploy only the independent cone observer and its fields to the idle console.

Backs up changed files and the service override, tests on the Pi, then restarts
only when all driving sessions remain disabled. Does not send drive commands.
"""
import argparse
import ast
import hashlib
import json
import shlex
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import paramiko
from deploy_brake_trial import command,put_atomic,read,ROOT,REMOTE,PYTHON,UNIT
from deploy_traffic_trial import idle as original_idle,PROTECTED as BASE_PROTECTED

sys.path.insert(0,str(ROOT/'vision/src'))
from carvision.brake_controls import extend_controls as brake
from carvision.traffic_controls import extend_controls as traffic
from carvision.cone_controls import extend_controls as cones

MODULES=('cone_vision','cone_road','cone_planning','cone_slalom','cone_navigation','cone_low_speed','cone_execution','cone_short_output','cone_short_trial','cone_observer','cone_controls')
FIXTURES=['vision/tests/fixtures/'+p.name for p in (ROOT/'vision/tests/fixtures').glob('cone*.jpg')]
FILES=['vision/src/carvision/'+n+'.py' for n in MODULES]+[
    'vision/configs/cone-trial.json','vision/configs/cone-ground-template.json','vision/configs/cone-primary-ground-reference.json','vision/tools/serve_crosswalk_trial.py',
    'vision/tools/cone_trial.py','vision/tests/test_cone_trial.py','vision/tests/test_cone_navigation.py','vision/tests/test_cone_short.py','vision/tests/test_cone_slalom.py','vision/tests/test_cone_low_speed.py','vision/CONE_TEST.md']+FIXTURES
SHORT_FILES=['vision/src/carvision/'+n+'.py' for n in ('cone_short_output','cone_short_trial','cone_observer','cone_controls')]+[
    'vision/tests/test_cone_short.py','vision/configs/cone-legacy-ground-candidate.json','vision/CONE_TEST.md']
SLALOM_FILES=['vision/src/carvision/'+n+'.py' for n in ('cone_planning','cone_slalom','cone_execution','cone_controls','cone_road','cone_navigation','cone_low_speed','cone_observer')]+[
    'vision/tests/test_cone_trial.py','vision/tests/test_cone_navigation.py','vision/tests/test_cone_slalom.py','vision/tests/test_cone_low_speed.py','vision/configs/cone-primary-ground-reference.json','vision/CONE_TEST.md']
PROTECTED=list(dict.fromkeys(BASE_PROTECTED+[
    'vision/src/carvision/'+n+'.py' for n in ('traffic_driving','traffic_trial','traffic_controls','traffic_signal',
        'traffic_voting','brake_output','autonomy_profile','config','lane','traffic_preview',
        'manual_drive','manual_hardware','steering_pwm_controls','parking_payment','payment_console','payment_controls',
        'departure','departure_trial','departure_controls','parking_bay','parking_bay_console','parking_bay_controls','parking_mission')]+[
    'vision/configs/traffic-driving.json','vision/configs/traffic-signal.json','vision/configs/cone-trial.local.json',
    'vision/configs/departure.local.json','vision/configs/parking-bay.json','vision/configs/parking-bay.local.json']))


def idle(status):
    c=status.get('cone_trial') or {}
    return (original_idle(status) and not (c.get('avoidance') or {}).get('active')
            and not (c.get('short_test') or {}).get('active')
            and not (status.get('departure_trial') or {}).get('active')
            and not (status.get('parking_payment') or {}).get('motion_locked')
            and not (status.get('parking_bay') or {}).get('motor_output_enabled')
            and not (status.get('parking_mission') or {}).get('active'))


def verify_protected(sftp, protected):
    for rel,data in protected.items():
        try:actual=read(sftp,REMOTE+'/'+rel)
        except FileNotFoundError:actual=None
        if actual!=data:raise RuntimeError('Protected control file changed: '+rel)


def extended_unit(data):
    lines=data.decode('utf-8').splitlines();found=0
    for i,line in enumerate(lines):
        if not line.startswith('ExecStart=') or line=='ExecStart=':continue
        argv=shlex.split(line[len('ExecStart='):])
        if (len(argv)<2 or argv[0]!=PYTHON or argv[1]!=REMOTE+'/vision/tools/serve_crosswalk_trial.py'
                or '--brake-config' not in argv or '--traffic-config' not in argv):
            raise ValueError('Unexpected active launcher; preserve it rather than overwrite')
        if '--cone-config' in argv:
            index=argv.index('--cone-config')
            if index+1>=len(argv):raise ValueError('Invalid cone option')
            argv[index+1]=REMOTE+'/vision/configs/cone-trial.json'
        else:argv+=['--cone-config',REMOTE+'/vision/configs/cone-trial.json']
        lines[i]='ExecStart='+shlex.join(argv);found+=1
    if found!=1:raise ValueError('Expected one service command')
    return ('\n'.join(lines)+'\n').encode()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handoff',type=Path,required=True)
    scope=parser.add_mutually_exclusive_group()
    scope.add_argument('--short-test-only',action='store_true',help='Only add fixed-PWM measurement entry; preserve launcher, existing cone config and every other control module')
    scope.add_argument('--slalom-only',action='store_true',help='Update only alternating cone logic and UI on an existing v2 console; preserve launcher, calibration and all other tasks')
    args=parser.parse_args()
    files=SLALOM_FILES if args.slalom_only else SHORT_FILES if args.short_test_only else FILES
    protected_files=list(PROTECTED)
    if args.short_test_only:
        protected_files+=['vision/tools/serve_crosswalk_trial.py','vision/configs/cone-trial.json']+[
            'vision/src/carvision/'+n+'.py' for n in ('cone_execution','cone_navigation','cone_planning','cone_vision','cone_road')]
    if args.slalom_only:
        protected_files+=['vision/tools/serve_crosswalk_trial.py','vision/tools/serve_parking_payment.py',
            'vision/configs/cone-trial.json','vision/configs/cone-legacy-ground-candidate.json']+[
            'vision/src/carvision/'+n+'.py' for n in ('cone_vision','cone_short_trial','cone_short_output')]
    cfg=json.loads((args.handoff/'connection.json').read_text(encoding='utf-8-sig'))['ssh']
    record=ROOT/'run'/('cone-console-deploy-'+time.strftime('%Y%m%d-%H%M%S'))
    record.mkdir(parents=True,exist_ok=False)
    audit={'deployed':False,'motion_commands_sent':False,'automatic_start':False}
    client=paramiko.SSHClient()
    client.load_host_keys(str(args.handoff/'attachments/known_hosts.smartcar'))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    changed=[];backup={};unit_changed=restarted=False
    def status():
        return json.loads(command(client,['curl','--fail','--silent','--max-time','3','http://127.0.0.1:8080/api/drive/status']))
    try:
        client.connect(cfg['host'],port=cfg.get('port',22),username=cfg['username'],password=cfg['password'],
            look_for_keys=False,allow_agent=False,timeout=6)
        sftp=client.open_sftp()
        before_status=status()
        if not idle(before_status):raise RuntimeError('Control active; no files or service changed')
        if args.slalom_only and (before_status.get('cone_trial') or {}).get('program')!='blue-cone-avoidance-v2':
            raise RuntimeError('Slalom-only update requires the existing cone v2 console')
        protected={}
        for rel in protected_files:
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
            if protected[rel] is not None:
                dst=record/'protected'/rel;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(protected[rel])
        tree=ast.parse(protected['vision/src/carvision/manual_controls.py'].decode('utf-8'))
        node=next(n.value for n in tree.body if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Name) and t.id=='SCRIPT' for t in n.targets))
        while isinstance(node,ast.Call):node=node.func.value
        controls=SimpleNamespace(SCRIPT=ast.literal_eval(node),HTML='')
        brake(controls);traffic(controls);cones(controls)
        (record/'extended-controls.js').write_text(controls.SCRIPT.removeprefix('<script>').removesuffix('</script>'),encoding='utf-8')
        before_unit=read(sftp,UNIT);unit=before_unit if args.short_test_only or args.slalom_only else extended_unit(before_unit)
        (record/'service.before.conf').write_bytes(before_unit)
        remote_backup=REMOTE+'/run/'+record.name
        command(client,['mkdir','-p',remote_backup]);put_atomic(sftp,remote_backup+'/service.before.conf',before_unit)
        for rel in files:
            try:backup[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:backup[rel]=None
            if backup[rel] is not None:
                dst=record/'before'/rel;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(backup[rel])
                command(client,['mkdir','-p',remote_backup+'/before/'+str(Path(rel).parent).replace('\\','/')])
                put_atomic(sftp,remote_backup+'/before/'+rel,backup[rel])
        for rel in files:
            command(client,['mkdir','-p',REMOTE+'/'+str(Path(rel).parent).replace('\\','/')])
            put_atomic(sftp,REMOTE+'/'+rel,(ROOT/rel).read_bytes());changed.append(rel)
        tests=['test_cone_trial.py','test_cone_navigation.py','test_cone_short.py','test_traffic_trial.py','test_traffic_driving.py','test_traffic_voting.py',
               'test_brake_trial.py','test_brake_parking_program.py','test_parking_speech.py']
        if not args.short_test_only:tests+=['test_cone_slalom.py','test_cone_low_speed.py']
        sample_path=remote_backup+'/cone-trial.repository.json'
        put_atomic(sftp,sample_path,(ROOT/'vision/configs/cone-trial.json').read_bytes())
        # Deployment tools belong to the PC. Provide reference sources only
        # inside this backup for offline predicates exercised by Pi tests.
        deploy_sources=remote_backup+'/deploy-test-sources'
        command(client,['mkdir','-p',deploy_sources])
        for name in ('deploy_traffic_trial.py','deploy_cone_trial.py'):
            put_atomic(sftp,deploy_sources+'/'+name,(ROOT/'vision/tools'/name).read_bytes())
        audit['pi_tests']=command(client,['env','PYTHONPATH='+REMOTE+'/vision/src',
            'CARVISION_REPOSITORY_CONE_CONFIG='+sample_path,'CARVISION_DEPLOY_SOURCE_DIR='+deploy_sources,PYTHON,'-m','pytest',
            *[REMOTE+'/vision/tests/'+n for n in tests],'-q'],timeout=60)
        if not idle(status()):raise RuntimeError('Control became active; no restart')
        put_atomic(sftp,remote_backup+'/service.new.conf',unit)
        if not idle(status()):raise RuntimeError('Control no longer idle; no restart')
        if read(sftp,UNIT)!=before_unit:raise RuntimeError('Service configuration changed during deployment; no restart')
        verify_protected(sftp,protected)
        if unit!=before_unit:
            unit_changed=True
            command(client,['install','-m','644',remote_backup+'/service.new.conf',UNIT],password=cfg['password'])
            command(client,['systemctl','daemon-reload'],password=cfg['password'])
        restarted=True
        command(client,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
        deadline=time.monotonic()+20
        while True:
            try:
                after=status()
                if (after.get('cone_trial') or {}).get('program')!='blue-cone-avoidance-v2':raise RuntimeError('Cone avoidance not loaded')
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.5)
        if not idle(after) or after['cone_trial']['hardware_output']:
            raise RuntimeError('Deployed console did not remain idle')
        if not args.short_test_only and not after['cone_trial']['avoidance'].get('slalom_required'):
            raise RuntimeError('Alternating cone enforcement not loaded')
        if after['cone_trial'].get('logic_revision')!='slalom-continuity-20261012':
            raise RuntimeError('Optimized cone logic not loaded')
        if (after['cone_trial'].get('short_test') or {}).get('program')!='cone-fixed-short-v1':raise RuntimeError('Short-test entry missing')
        verify_protected(sftp,protected)
        for rel in files:
            if read(sftp,REMOTE+'/'+rel)!=(ROOT/rel).read_bytes():raise RuntimeError('Uploaded file mismatch: '+rel)
        audit.update(deployed=True,backup=remote_backup,protected_files_unchanged=True,
            geometry=after['cone_trial']['settings']['geometry'],
            after={key:after.get(key) for key in ('mode','hardware_output','worker_started','boot_test_phase')},
            short_test_only=args.short_test_only,slalom_only=args.slalom_only,service_configuration_unchanged=read(sftp,UNIT)==before_unit,
            sha256={rel:hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() for rel in files})
    except Exception as exc:
        audit['error']=str(exc)
        if changed:
            try:
                for rel in reversed(changed):
                    if backup[rel] is None:sftp.remove(REMOTE+'/'+rel)
                    else:put_atomic(sftp,REMOTE+'/'+rel,backup[rel])
                if unit_changed:
                    command(client,['install','-m','644',remote_backup+'/service.before.conf',UNIT],password=cfg['password'])
                    command(client,['systemctl','daemon-reload'],password=cfg['password'])
                if restarted:command(client,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
                audit['rolled_back']=True
            except Exception as rollback_error:audit['rollback_error']=str(rollback_error)
    finally:
        client.close();(record/'deployment.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in audit.items() if k!='sha256'},ensure_ascii=False,indent=2))
    return 0 if audit['deployed'] else 1


if __name__=='__main__':raise SystemExit(main())
