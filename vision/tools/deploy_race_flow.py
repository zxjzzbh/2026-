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
from pathlib import Path

import paramiko
from deploy_brake_trial import command,put_atomic,read,ROOT,REMOTE,PYTHON,UNIT
from deploy_cone_trial import idle,PROTECTED,verify_protected

MODULES=('race_flow','race_flow_camera','race_flow_console','race_flow_controls',
         'parking_bay','parking_bay_console','parking_bay_controls','parking_mission')
FILES=['vision/src/carvision/'+name+'.py' for name in MODULES]+[
    'vision/tools/serve_race_flow.py','vision/tools/serve_parking_payment.py','vision/tools/race_flow.py',
    'vision/configs/race-flow.json','vision/configs/parking-bay.json','vision/tests/test_race_flow.py',
    'vision/tests/test_parking_bay.py','vision/tests/fixtures/cone-track-short-secondary.jpg',
    'vision/RACE_FLOW.md','vision/PARKING_BAY.md']
PREREQUISITES=['vision/src/carvision/'+name+'.py' for name in (
    'cone_low_speed','cone_observer','cone_trial_output','cone_boundaries','cone_road','cone_navigation','cone_planning')]


def flow_idle(status):
    return idle(status) and not (status.get('race_flow') or {}).get('active')


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
    changed=[];backup={};desired={};restarted=False;unit_changed=False;audit={'deployed':False,'motion_commands_sent':False}
    def status():return json.loads(command(client,['curl','--fail','--silent','--max-time','3','http://127.0.0.1:8080/api/drive/status']))
    try:
        client.connect(cfg['host'],port=cfg.get('port',22),username=cfg['username'],password=cfg['password'],
                       look_for_keys=False,allow_agent=False,timeout=6)
        if not flow_idle(status()):raise RuntimeError('当前仍有车辆控制会话，未修改任何文件')
        sftp=client.open_sftp()
        for rel in PREREQUISITES:
            if read(sftp,REMOTE+'/'+rel)!=(ROOT/rel).read_bytes():
                raise RuntimeError('车端锥桶基线不同，停止部署以免覆盖队友更新：'+rel)
        protected={}
        for rel in PROTECTED:
            if rel in FILES:continue
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
        for rel in ('vision/configs/race-flow.local.json','vision/configs/parking-bay.local.json'):
            try:protected[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:protected[rel]=None
        unit_before=read(sftp,UNIT);unit_after=extend_unit(unit_before)
        (record/'service.before.conf').write_bytes(unit_before)
        remote_backup=REMOTE+'/run/'+record.name;command(client,['mkdir','-p',remote_backup])
        put_atomic(sftp,remote_backup+'/service.before.conf',unit_before)
        for rel in FILES:
            try:backup[rel]=read(sftp,REMOTE+'/'+rel)
            except FileNotFoundError:backup[rel]=None
            if backup[rel] is not None:
                local=record/'before'/rel;local.parent.mkdir(parents=True,exist_ok=True);local.write_bytes(backup[rel])
                command(client,['mkdir','-p',remote_backup+'/before/'+str(Path(rel).parent).replace('\\','/')])
                put_atomic(sftp,remote_backup+'/before/'+rel,backup[rel])
        for rel in FILES:
            command(client,['mkdir','-p',REMOTE+'/'+str(Path(rel).parent).replace('\\','/')])
            desired[rel]=backup[rel] if rel.startswith('vision/configs/') and backup[rel] is not None else (ROOT/rel).read_bytes()
            put_atomic(sftp,REMOTE+'/'+rel,desired[rel]);changed.append(rel)
        test_root=remote_backup+'/test-root';command(client,['mkdir','-p',test_root+'/vision/configs'])
        for name in ('race-flow','brake-parking','traffic-driving','cone-trial','parking-bay','cone-legacy-ground-candidate'):
            path=ROOT/'vision/configs'/(name+'.json')
            put_atomic(sftp,test_root+'/vision/configs/'+path.name,path.read_bytes())
        audit['tests']=command(client,['env','PYTHONPATH='+REMOTE+'/vision/src','CARVISION_FLOW_TEST_ROOT='+test_root,
            'CARVISION_SKIP_PC_DEPLOY_TESTS=1',
            PYTHON,'-m','pytest',REMOTE+'/vision/tests/test_race_flow.py',REMOTE+'/vision/tests/test_parking_bay.py','-q'],timeout=60)
        if not flow_idle(status()):raise RuntimeError('控制已启用，禁止重启')
        verify_protected(sftp,protected)
        if read(sftp,UNIT)!=unit_before:raise RuntimeError('启动配置被其他任务修改，停止部署')
        put_atomic(sftp,remote_backup+'/service.new.conf',unit_after)
        unit_changed=True;command(client,['install','-m','644',remote_backup+'/service.new.conf',UNIT],password=cfg['password'])
        command(client,['systemctl','daemon-reload'],password=cfg['password'])
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
                     controller_stage=state['race_flow']['stage'],parking_missing=state['race_flow']['parking_missing'])
    except Exception as exc:
        audit['error']=str(exc)
        if changed:
            for rel in reversed(changed):
                if backup[rel] is None:
                    try:sftp.remove(REMOTE+'/'+rel)
                    except FileNotFoundError:pass
                else:put_atomic(sftp,REMOTE+'/'+rel,backup[rel])
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
