"""Public manual construction owns workers without importing launch scripts."""
import ast
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from carvision import manual_service
from carvision.worker_client import OwnedWorkerDrive


VISION=Path(__file__).resolve().parents[1]


def load_tool(filename):
    spec=importlib.util.spec_from_file_location('compat_'+filename[:-3],VISION/'tools'/filename)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_legacy_launchers_alias_the_same_public_functions_and_class():
    boot=load_tool('serve_boot_test.py');bench=load_tool('serve_bench_drive.py')
    assert boot.initial_status is manual_service.initial_status
    assert boot.repeated_drive is manual_service.repeated_drive
    assert boot.OwnedWorkerDrive is bench.OwnedWorkerDrive is OwnedWorkerDrive
    for filename,names in [('serve_boot_test.py',{'initial_status','repeated_drive'}),
                           ('serve_bench_drive.py',{'OwnedWorkerDrive'})]:
        tree=ast.parse((VISION/'tools'/filename).read_text(encoding='utf-8'))
        assert not any(isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name in names for node in tree.body)


def test_public_construction_imports_without_tools_path_or_site_packages():
    code=("import sys; import carvision.manual_service, carvision.worker_client; "
          "assert not any(name.startswith('serve_') for name in sys.modules); "
          "assert 'cv2' not in sys.modules and 'numpy' not in sys.modules")
    result=subprocess.run([sys.executable,'-S','-c',code],
        env={**os.environ,'PYTHONPATH':str(VISION/'src')},capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize('continuous',[False,True])
@pytest.mark.parametrize('allow_handoff',[False,True])
def test_shared_factory_preserves_worker_command_and_explicit_handoff_flag(tmp_path,monkeypatch,continuous,allow_handoff):
    folder=tmp_path/'prepared';folder.mkdir();endpoint=tmp_path/'fake-endpoint';endpoint.touch()
    review={'ground_continuous_requested_by_user':continuous};seen={}
    context={'review':review,'boot_id':'current-boot','folder':folder}
    monkeypatch.setattr(manual_service,'reviewed_manifest',lambda root,data:seen.update(review_checked=data))
    monkeypatch.setattr(manual_service,'worker_control_endpoint',lambda output:endpoint)
    original_read=Path.read_text
    def read(path,*args,**kwargs):
        if path.as_posix()=='/proc/sys/kernel/random/boot_id':return 'current-boot'
        return original_read(path,*args,**kwargs)
    monkeypatch.setattr(Path,'read_text',read)
    process=SimpleNamespace(poll=lambda:None)
    def popen(command,**kwargs):seen.update(command=command,popen=kwargs);return process
    monkeypatch.setattr(manual_service.subprocess,'Popen',popen)
    def owned(path,**kwargs):seen.update(endpoint=path,owner=kwargs);return SimpleNamespace(**kwargs)
    monkeypatch.setattr(manual_service,'OwnedWorkerDrive',owned)
    drive=manual_service.repeated_drive(tmp_path,context,allow_autonomy_handoff=allow_handoff)
    assert 'command' not in seen  # Construction/status do not launch hardware.
    drive.status();assert 'command' not in seen
    worker=drive.factory()
    try:
        command=seen['command']
        assert command[:7]==['timeout','--signal=TERM','--kill-after=1s','0s' if continuous else '810s',
                            '/usr/bin/python3','-m','carvision.manual_hardware']
        assert '--ground-continuous' in command if continuous else '--ground-short' in command
        assert '--ground-prepared' in command and '--pi5-review' in command
        assert ('--allow-autonomy-handoff' in command)==allow_handoff
        assert seen['owner']['process'] is process and seen['owner']['expected_boot']=='current-boot'
        assert seen['popen']['start_new_session'] is True
        assert seen['popen']['env']['PYTHONPATH']==str(tmp_path/'vision/src')
        with pytest.raises(RuntimeError,match='尚未清理'):drive.factory()
    finally:worker.worker_log.close()
