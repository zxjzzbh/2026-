"""Optional explicit deployment; default is LOCAL CHECK ONLY, no SSH.

--apply is only for a later authorized car update with all control sessions
released. Credentials come from the existing private handoff, never the repo.
"""
import argparse
import hashlib
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path,PurePosixPath

import paramiko
from deploy_brake_trial import command,put_atomic,read,ROOT,REMOTE,PYTHON,UNIT
from deploy_cone_trial import idle,PROTECTED,verify_protected

MODULES=('race_flow','race_flow_camera','race_flow_console','race_flow_controls',
         'parking_bay','parking_bay_console','parking_bay_controls','parking_mission','parking_entry','parking_entry_run','parking_guidance','parking_path','parking_control','parking_feedback','parking_far_tracker',
         'steering','manual_drive','manual_hardware','rasadapter5','pi5_pwm',
         'brake_parking','brake_output','cone_low_speed','cone_continuous','cone_trial_output','cone_navigation','cone_execution',
         'cone_planning','cone_observer','cone_controls','autonomy_profile','autonomy_motion','rasadapter5_executor')
FILES=['vision/src/carvision/'+name+'.py' for name in MODULES]+[
    'vision/tools/serve_race_flow.py','vision/tools/serve_parking_payment.py','vision/tools/race_flow.py',
    'vision/configs/race-flow.json','vision/configs/parking-bay.json','vision/tests/test_race_flow.py',
    'vision/tests/test_parking_bay.py','vision/tests/test_parking_path.py','vision/tests/test_parking_entry.py','vision/tests/test_parking_entry_run.py','vision/tests/test_parking_guidance.py','vision/tests/test_parking_direction_hint.py','vision/tests/test_parking_washed_board.py',
    'vision/tests/test_parking_far_tracker.py','vision/tests/test_parking_boundary_guard.py',
    'vision/tests/fixtures/parking-overshoot-loss-20261013.json','vision/tests/fixtures/parking-boundary-loss-20261013.json',
    'vision/tests/test_cone_low_speed.py','vision/tests/test_cone_trial_output.py',
    'vision/tests/test_cone_continuous.py','vision/tests/test_cone_slalom.py',
    'vision/tests/fixtures/cone-track-short-secondary.jpg','vision/tests/fixtures/cone-track-glare-primary.jpg',
    'vision/tests/fixtures/cone-track-glare-secondary.jpg',
    'vision/tests/fixtures/cone-continuity-20261013.json',
    'vision/tests/fixtures/parking-pale-green-left-board-20261013.jpg',
    'vision/tests/fixtures/parking-border-clipped-left-board-20261013.jpg',
    'vision/tests/fixtures/parking-washed-right-board-20261013.jpg',
    'vision/RACE_FLOW.md','vision/PARKING_BAY.md','vision/PARKING_PATH.md']
PREREQUISITES=['vision/src/carvision/'+name+'.py' for name in (
    'cone_boundaries','cone_road')]


def preserved_runtime(rel,data):
    if data is None:return False
    manifest=json.loads((ROOT/'vision/configs/race-upgrade-source.json').read_text(encoding='utf-8'))
    return hashlib.sha256(data).hexdigest() in manifest.get('preserved_runtime_hashes',{}).get(rel,[])


def compatible_source(rel,data):
    if data is None or data==(ROOT/rel).read_bytes():return True
    if preserved_runtime(rel,data):return True
    manifest=json.loads((ROOT/'vision/configs/race-upgrade-source.json').read_text(encoding='utf-8'))
    return hashlib.sha256(data).hexdigest() in manifest['predecessor_hashes'].get(rel,[])


def flow_idle(status):
    # A failed software-manifest check occurs before the boot helper opens an
    # actuator. It may be serviced without inventing an ESC-off confirmation.
    released_software_fault=(status.get('mode')=='fault' and status.get('boot_test_phase')=='fault'
        and status.get('reason')=='已检查的软件发生变化，请先重新核验'
        and all(status.get(k) is False for k in ('hardware_output','worker_started','pending_hardware_start'))
        and all(not (status.get(k) or {}).get('active') for k in ('crosswalk_trial','traffic_trial','departure_trial','parking_trial'))
        and all(not ((status.get('cone_trial') or {}).get(k) or {}).get('active') for k in ('avoidance','short_test'))
        and not (status.get('parking_payment') or {}).get('motion_locked')
        and not (status.get('parking_bay') or {}).get('motor_output_enabled'))
    return (idle(status) or released_software_fault) and not (status.get('race_flow') or {}).get('active')


def updated_checks(before,hashes,test_output):
    """Refresh only this deployment's reviewed files after real checks pass."""
    updated=json.loads(json.dumps(before));allowed=set(FILES)|{'vision/configs/cone-trial.local.json'}
    for item in updated['files']:
        rel=item['path'];actual=hashes[rel]
        if actual!=item['sha256'] and rel not in allowed:
            raise RuntimeError('软件核验发现范围外文件变化，不自动认可：'+rel)
        item['sha256']=actual
    updated['previous_pi_checks']=before['pi_checks']
    updated['pi_checks']={'rc':0,'stdout':test_output,'scope':'deployed task regressions and preserved manual runtime regressions'}
    updated['race_deployment_recheck']={'motion_commands_sent':False,'physical_driving_retested':False,'checked_at_epoch_s':time.time()}
    return updated


def remote_hashes(client,paths):
    script="""import sys,json,hashlib
from pathlib import Path
root=Path(sys.argv[1]).resolve();result={}
for rel in sys.argv[2:]:
 p=(root/rel).resolve()
 if not p.is_relative_to(root):raise ValueError('check path outside workspace')
 result[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
print(json.dumps(result))
"""
    return json.loads(command(client,[PYTHON,'-c',script,REMOTE,*paths]))


def extend_unit(data):
    lines=data.decode('utf-8').splitlines();count=0
    for i,line in enumerate(lines):
        if not line.startswith('ExecStart=') or line=='ExecStart=':continue
        argv=shlex.split(line[len('ExecStart='):])
        if len(argv)<2 or argv[0]!=PYTHON or argv[1] not in (
                REMOTE+'/vision/tools/serve_parking_payment.py',REMOTE+'/vision/tools/serve_race_flow.py'):
            raise ValueError('启动入口不是已知的原组合控制台，需先核对，不覆盖未知启动方式')
        argv[1]=REMOTE+'/vision/tools/serve_race_flow.py'
        if '--race-live' not in argv:argv.append('--race-live')
        lines[i]='ExecStart='+shlex.join(argv);count+=1
    if count!=1:raise ValueError('需要唯一有效 ExecStart')
    return ('\n'.join(lines)+'\n').encode()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true',help='Explicitly connect, back up and deploy to the idle car')
    parser.add_argument('--handoff',type=Path,default=ROOT.parent/'连接准备_20261009/smartcar-connection-handoff')
    args=parser.parse_args(argv)
    if not args.apply:
        return subprocess.call([sys.executable,str(ROOT/'vision/tools/race_flow.py'),'check'])
    cfg=json.loads((args.handoff/'connection.json').read_text(encoding='utf-8-sig'))['ssh']
    record=ROOT/'run'/('race-flow-deploy-'+str(time.time_ns()));record.mkdir(parents=True)
    client=paramiko.SSHClient();client.load_host_keys(str(args.handoff/'attachments/known_hosts.smartcar'))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    changed=[];backup={};desired={};restarted=False;unit_changed=False;checks_changed=False
    checks_before=checks_path=None;audit={'deployed':False,'motion_commands_sent':False}
    def status():return json.loads(command(client,['curl','--fail','--silent','--max-time','3','http://127.0.0.1:8080/api/drive/status']))
    try:
        client.connect(cfg['host'],port=cfg.get('port',22),username=cfg['username'],password=cfg['password'],
                       look_for_keys=False,allow_agent=False,timeout=6,banner_timeout=6,auth_timeout=6)
        client.get_transport().set_keepalive(15)
        if not flow_idle(status()):raise RuntimeError('当前仍有车辆控制会话，未修改任何文件')
        print('Idle controller verified; checking source versions.',flush=True)
        sftp=client.open_sftp()
        for rel in PREREQUISITES:
            if read(sftp,REMOTE+'/'+rel)!=(ROOT/rel).read_bytes():
                raise RuntimeError('车端锥桶基线不同，停止部署以免覆盖队友更新：'+rel)
        for rel in FILES:
            if not rel.startswith('vision/src/'):continue
            try:data=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:data=None
            if not compatible_source(rel,data):
                raise RuntimeError('车端控制源码有未对齐更新，不覆盖：'+rel)
        protected={}
        manifest=json.loads((ROOT/'vision/configs/race-upgrade-source.json').read_text(encoding='utf-8'))
        for rel in manifest.get('preserved_runtime_hashes',{}):
            # Newer installed manual-control dependencies stay byte-identical.
            data=read(sftp,REMOTE+'/'+rel)
            if not preserved_runtime(rel,data):raise RuntimeError('已审查的车端遥控依赖发生变化：'+rel)
            protected[rel]=data
        for rel in PROTECTED:
            if rel in FILES:continue
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
        for rel in ('vision/configs/race-flow.local.json','vision/configs/parking-bay.local.json'):
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
        unit_before=read(sftp,UNIT);unit_after=extend_unit(unit_before)
        template=json.loads(read(sftp,REMOTE+'/vision/deployment/boot-test-template.json'))
        checks_rel=template['software_checks_record'];check_path=PurePosixPath(checks_rel)
        if check_path.is_absolute() or '..' in check_path.parts:raise ValueError('软件核验路径必须位于当前仓库')
        checks_path=REMOTE+'/'+checks_rel;checks_before=read(sftp,checks_path)
        (record/'boot-checks.before.json').write_bytes(checks_before)
        (record/'service.before.conf').write_bytes(unit_before)
        remote_backup=REMOTE+'/run/'+record.name
        # One directory command avoids dozens of SSH session round trips on 5G.
        parents={str(Path(rel).parent).replace('\\','/') for rel in FILES}
        directories=[remote_backup]+[remote_backup+'/before/'+p for p in sorted(parents)]+[REMOTE+'/'+p for p in sorted(parents)]
        command(client,['mkdir','-p',*directories])
        put_atomic(sftp,remote_backup+'/service.before.conf',unit_before)
        print('Backing up the installed files.',flush=True)
        for rel in FILES:
            try:backup[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:backup[rel]=None
            if backup[rel] is not None:
                local=record/'before'/rel;local.parent.mkdir(parents=True,exist_ok=True);local.write_bytes(backup[rel])
                put_atomic(sftp,remote_backup+'/before/'+rel,backup[rel])
        print('Synchronizing changed files; installed manual runtime is preserved.',flush=True)
        for rel in FILES:
            keep=(rel.startswith('vision/configs/') and backup[rel] is not None) or preserved_runtime(rel,backup[rel])
            desired[rel]=backup[rel] if keep else (ROOT/rel).read_bytes()
            if desired[rel]!=backup[rel]:
                put_atomic(sftp,REMOTE+'/'+rel,desired[rel]);changed.append(rel)
        test_root=remote_backup+'/test-root';command(client,['mkdir','-p',test_root+'/vision/configs'])
        for name in ('race-flow','brake-parking','traffic-driving','cone-trial','parking-bay','cone-legacy-ground-candidate'):
            path=ROOT/'vision/configs'/(name+'.json')
            put_atomic(sftp,test_root+'/vision/configs/'+path.name,path.read_bytes())
        print('Running software-only regressions on the car.',flush=True)
        audit['tests']=command(client,['env','PYTHONPATH='+REMOTE+'/vision/src','CARVISION_FLOW_TEST_ROOT='+test_root,
            'CARVISION_SKIP_PC_DEPLOY_TESTS=1',
            'CARVISION_REPOSITORY_CONE_CONFIG='+test_root+'/vision/configs/cone-trial.json',
            PYTHON,'-m','pytest',REMOTE+'/vision/tests/test_race_flow.py',REMOTE+'/vision/tests/test_parking_bay.py',
            REMOTE+'/vision/tests/test_parking_path.py',REMOTE+'/vision/tests/test_parking_entry.py',REMOTE+'/vision/tests/test_parking_entry_run.py',
            REMOTE+'/vision/tests/test_parking_far_tracker.py',REMOTE+'/vision/tests/test_parking_boundary_guard.py',
            REMOTE+'/vision/tests/test_parking_guidance.py',
            REMOTE+'/vision/tests/test_parking_direction_hint.py',
            REMOTE+'/vision/tests/test_parking_washed_board.py',
            REMOTE+'/vision/tests/test_cone_low_speed.py',REMOTE+'/vision/tests/test_cone_continuous.py',
            REMOTE+'/vision/tests/test_cone_trial_output.py','-q'],timeout=60)
        if manifest.get('preserved_runtime_hashes'):
            audit['manual_regression_tests']=command(client,['env','PYTHONPATH='+REMOTE+'/vision/src',
                'CARVISION_REPOSITORY_CONE_CONFIG='+test_root+'/vision/configs/cone-trial.json',PYTHON,'-m','pytest',
                *[REMOTE+'/vision/tests/'+name+'.py' for name in ('test_ground_continuous_drive','test_symmetric_manual_steering',
                    'test_current_steering_profile','test_manual_heartbeat')],'-q'],timeout=60)
        if not flow_idle(status()):raise RuntimeError('控制已启用，禁止重启')
        verify_protected(sftp,protected)
        # UI-written cone parameters are schema-checked, not falsely promoted
        # to field calibration. Keep all physical confirmation fields intact.
        config_check="from carvision.cone_planning import load_settings; from carvision.cone_navigation import validate_driving; from pathlib import Path; import sys; p=Path(sys.argv[1]); validate_driving(load_settings(p)) if p.exists() else None; print('configuration schema checked')"
        command(client,['env','PYTHONPATH='+REMOTE+'/vision/src',PYTHON,'-c',config_check,REMOTE+'/vision/configs/cone-trial.local.json'])
        previous_checks=json.loads(checks_before)
        hashes=remote_hashes(client,[item['path'] for item in previous_checks['files']])
        checked=updated_checks(previous_checks,hashes,audit['tests']+'\n'+audit.get('manual_regression_tests',''))
        put_atomic(sftp,remote_backup+'/boot-checks.before.json',checks_before)
        checks_changed=True;put_atomic(sftp,checks_path,json.dumps(checked,ensure_ascii=False,indent=2).encode('utf-8'))
        verify_script="import json,sys; from pathlib import Path; from carvision.boot_test import reviewed_manifest; root=Path(sys.argv[1]); reviewed_manifest(root,json.loads((root/'vision/deployment/boot-test-template.json').read_text())); print('manual software manifest verified')"
        audit['software_manifest_rechecked']=command(client,['env','PYTHONPATH='+REMOTE+'/vision/src',PYTHON,'-c',verify_script,REMOTE]).strip()
        if read(sftp,UNIT)!=unit_before:raise RuntimeError('启动配置被其他任务修改，停止部署')
        put_atomic(sftp,remote_backup+'/service.new.conf',unit_after)
        unit_changed=True;command(client,['install','-m','644',remote_backup+'/service.new.conf',UNIT],password=cfg['password'])
        command(client,['systemctl','daemon-reload'],password=cfg['password'])
        print('Tests passed; restarting the idle preview service.',flush=True)
        restarted=True;command(client,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
        deadline=time.monotonic()+20
        while True:
            try:
                state=status()
                if 'race_flow' not in state:raise ValueError('新总流程模块未加载')
                break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.5)
        if not flow_idle(state) or state['hardware_output']:raise RuntimeError('更新后车辆未保持控制关闭')
        if state['race_flow'].get('configuration_error') or not state['race_flow'].get('live_enabled'):
            raise RuntimeError('总流程配置未就绪：'+state['race_flow'].get('reason','unknown'))
        verify_protected(sftp,protected)
        for rel in FILES:
            if read(sftp,REMOTE+'/'+rel)!=desired[rel]:raise RuntimeError('文件校验不一致：'+rel)
        audit.update(deployed=True,backup=remote_backup,protected_files_unchanged=True,
                     preserved_runtime=list(manifest.get('preserved_runtime_hashes',{})),
                     controller_stage=state['race_flow']['stage'],parking_missing=state['race_flow']['parking_missing'])
    except Exception as exc:
        audit['error']=str(exc)
        if changed or checks_changed:
            for rel in reversed(changed):
                if backup[rel] is None:
                    try:sftp.remove(REMOTE+'/'+rel)
                    except FileNotFoundError:pass
                else:put_atomic(sftp,REMOTE+'/'+rel,backup[rel])
            if checks_changed:put_atomic(sftp,checks_path,checks_before)
            if unit_changed:
                command(client,['install','-m','644',remote_backup+'/service.before.conf',UNIT],password=cfg['password'])
                command(client,['systemctl','daemon-reload'],password=cfg['password'])
            if restarted:command(client,['systemctl','restart','smartcar-preview.service'],password=cfg['password'])
            audit['rolled_back']=True
        raise
    finally:
        client.close();(record/'deployment.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(audit,ensure_ascii=False,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
