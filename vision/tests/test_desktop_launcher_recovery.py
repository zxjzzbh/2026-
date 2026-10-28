"""Offline desktop service-switch failure recovery; no SSH/browser is opened."""
import io
from pathlib import Path
from types import ModuleType

import pytest


HERE=Path(__file__).resolve().parent
LAUNCHER=HERE/'desktop_delivery/launch_vehicle.py'
if not LAUNCHER.exists():
    # The same tests can be included under candidate/vision/tests.
    LAUNCHER=HERE.parents[1]/'pc-launcher/launch_vehicle.py'
launcher=ModuleType('desktop_launcher_final_review')
launcher.__file__=str(LAUNCHER)
exec(compile(LAUNCHER.read_text(encoding='utf-8'),str(LAUNCHER),'exec'),launcher.__dict__)


def idle():
    return {'application_mode':'debug','mode':'disabled','boot_test_phase':'esc_off',
            'hardware_output':False,'worker_started':False,'pending_hardware_start':False}


class RemoteFiles:
    def __init__(self,previous):
        self.files={} if previous is None else {launcher.DROP_IN:previous}
    def __enter__(self):return self
    def __exit__(self,*args):pass
    def open(self,path,mode):
        if mode=='rb':
            if path not in self.files:raise FileNotFoundError(path)
            return io.BytesIO(self.files[path])
        remote=self
        class Writer(io.BytesIO):
            def close(self):
                if not self.closed:remote.files[path]=self.getvalue()
                super().close()
        return Writer()
    def remove(self,path):
        if path not in self.files:raise FileNotFoundError(path)
        del self.files[path]


class Client:
    def __init__(self,previous):self.remote=RemoteFiles(previous);self.opened=False
    def open_sftp(self):self.opened=True;return self.remote


def fake_commands(monkeypatch,client,*,fail_stop=False,fail_restore=False,partial_target=True):
    state={'running':True,'commands':[],'install_count':0}
    def command(_client,args,**kwargs):
        assert _client is client
        state['commands'].append(tuple(args))
        if args[:2]==['systemctl','stop']:
            if fail_stop:raise RuntimeError('simulated stop failure')
            state['running']=False
        elif args[0]=='install':
            state['install_count']+=1
            if state['install_count']==1:
                # The failed install may already have overwritten the target.
                if partial_target:client.remote.files[launcher.DROP_IN]=b'partial replacement'
                raise RuntimeError('simulated first install failure')
            if fail_restore:raise RuntimeError('simulated rollback install failure')
            client.remote.files[launcher.DROP_IN]=client.remote.files[args[-2]]
        elif args[0]=='rm':
            if launcher.DROP_IN not in client.remote.files and '-f' not in args:
                raise RuntimeError('missing target cannot be removed without -f')
            client.remote.files.pop(launcher.DROP_IN,None)
        elif args[:2]==['systemctl','start']:state['running']=True
        return ''
    monkeypatch.setattr(launcher,'command',command)
    monkeypatch.setattr(launcher,'read_state',lambda _client:idle())
    return state


@pytest.mark.parametrize('previous',[None,b'[Service]\nExecStart=old service\n'])
@pytest.mark.parametrize('partial_target',[False,True])
def test_failed_install_restores_previous_dropin_and_restarts_original_service(monkeypatch,previous,partial_target):
    client=Client(previous);state=fake_commands(monkeypatch,client,partial_target=partial_target)
    with pytest.raises(RuntimeError,match='first install failure'):
        launcher.select_mode(client,{'password':'test-only'},'race')
    assert state['running']
    assert client.remote.files.get(launcher.DROP_IN)==previous
    assert ('systemctl','daemon-reload') in state['commands']
    assert state['commands'][-1]==('systemctl','start',launcher.UNIT)
    if previous is None:assert ('rm','-f','--',launcher.DROP_IN) in state['commands']
    assert list(client.remote.files)==([] if previous is None else [launcher.DROP_IN])


def test_failed_stop_never_installs_or_removes_mode_configuration(monkeypatch):
    previous=b'unchanged old config';client=Client(previous)
    state=fake_commands(monkeypatch,client,fail_stop=True)
    with pytest.raises(RuntimeError,match='stop failure'):
        launcher.select_mode(client,{'password':'test-only'},'race')
    assert state['commands']==[('systemctl','stop',launcher.UNIT)]
    assert state['running'] and client.remote.files=={launcher.DROP_IN:previous}


def test_rollback_failure_is_reported_without_starting_partial_configuration(monkeypatch):
    client=Client(b'original config');state=fake_commands(monkeypatch,client,fail_restore=True)
    with pytest.raises(RuntimeError,match='恢复失败'):
        launcher.select_mode(client,{'password':'test-only'},'race')
    assert not state['running']
    assert ('systemctl','start',launcher.UNIT) not in state['commands']


def test_same_mode_active_page_is_opened_without_restarting_vehicle(monkeypatch):
    client=Client(None);active={**idle(),'mode':'enabled','hardware_output':True}
    monkeypatch.setattr(launcher,'read_state',lambda _client:active)
    monkeypatch.setattr(launcher,'command',lambda *args,**kwargs:pytest.fail('active service was changed'))
    assert launcher.select_mode(client,{},'debug') is active and not client.opened


def test_other_mode_active_vehicle_is_rejected_before_any_remote_write(monkeypatch):
    client=Client(None);active={**idle(),'mode':'enabled','hardware_output':True}
    monkeypatch.setattr(launcher,'read_state',lambda _client:active)
    monkeypatch.setattr(launcher,'command',lambda *args,**kwargs:pytest.fail('active service was changed'))
    with pytest.raises(RuntimeError,match='正在准备'):launcher.select_mode(client,{},'race')
    assert not client.opened and client.remote.files=={}
