"""Flat application ownership/cleanup with fake adapters; no sockets or devices."""
import threading
import json
from types import SimpleNamespace

import pytest

from carvision.application import DebugApplication,CompetitionServices,_finish_all


class Manual:
    def __init__(self):self.calls=[];self.closed=False
    def status(self):return {'mode':'disabled','boot_test_phase':'esc_off','owner_client':'owner'}
    def request(self,action,payload):self.calls.append(action)
    def close(self):self.closed=True


class Task:
    def __init__(self,field,*,active=False,fail_stop=False,fail_close=False):
        self.field,self.active,self.fail_stop,self.fail_close=field,active,fail_stop,fail_close
        self.calls=[];self.closed=False
    def status(self):return {self.field:{'active':self.active,'owner_client':'owner'}}
    def request(self,action,payload):
        self.calls.append(action)
        if self.fail_stop:raise ValueError('adapter stop failed')
        self.active=False
    def close(self):
        self.closed=True
        if self.fail_close:raise RuntimeError('adapter close failed')


def application():
    app=DebugApplication.__new__(DebugApplication)
    app.manual=Manual();app.lock=threading.RLock();app.extensions={}
    app.closed=False;app.stop_errors=[];app.close_errors=[];app.prefixes={}
    app.tasks={'brake':Task('crosswalk_trial',fail_stop=True,fail_close=True),
               'traffic':Task('traffic_trial',active=True)}
    return app


@pytest.mark.parametrize('action',['stop','finish_test','emergency'])
def test_one_adapter_failure_does_not_skip_the_active_task_stop(action):
    app=application()
    status=app.request(action,{'client':'owner'})
    assert app.tasks['brake'].calls and app.tasks['traffic'].calls
    assert not app.tasks['traffic'].active
    assert status['stop_errors']==[{'component':'brake','error':'adapter stop failed'}]
    if action!='finish_test':assert app.manual.calls==[action]


def test_other_page_ordinary_stop_is_ignored_but_emergency_always_reaches_tasks():
    app=application()
    app.request('stop',{'client':'another'})
    assert not app.manual.calls and not app.tasks['traffic'].calls
    app.request('emergency',{'client':'another'})
    assert app.manual.calls==['emergency'] and app.tasks['traffic'].calls==['emergency']


def test_close_reaches_manual_and_all_tasks_even_if_one_close_fails():
    app=application();app.close()
    assert app.manual.closed and all(task.closed for task in app.tasks.values())
    assert app.close_errors==[{'component':'brake','error':'adapter close failed'}]
    app.close()
    with pytest.raises(RuntimeError,match='已关闭'):app.request('enable',{})


def test_final_cleanup_attempts_all_resources_after_a_failure():
    finished=[]
    def fail():raise RuntimeError('close failed')
    errors=_finish_all([('application',fail),('services',lambda:finished.append('services')),
                        ('camera',lambda:finished.append('camera')),('server',lambda:finished.append('server'))])
    assert finished==['services','camera','server']
    assert errors==[{'component':'application','error':'close failed'}]


def test_competition_services_close_the_original_manual_worker_if_scanner_close_fails():
    service=CompetitionServices.__new__(CompetitionServices)
    service.manual=Manual();service.payment=Task('parking_payment',fail_close=True)
    service.closed=False;service.close_errors=[]
    service.close()
    assert service.manual.closed and service.payment.closed
    assert service.close_errors==[{'component':'payment','error':'adapter close failed'}]
    service.close()


def test_application_shutdown_closes_race_services_and_both_cameras_after_app_error(tmp_path,monkeypatch):
    import carvision.application as module
    import carvision.competition_page as pages
    calls=[]
    config=tmp_path/'vision/configs';config.mkdir(parents=True)
    (config/'cameras.json').write_text(json.dumps({'primary':'fake-primary','secondary':'fake-secondary'}))
    class Resource:
        def __init__(self,name,fail=False):self.name,self.fail=name,fail
        def close(self):
            calls.append(self.name)
            if self.fail:raise RuntimeError('application close failed')
    class Worker(Resource):
        def __init__(self,state,config,options):super().__init__(options.camera)
        def start(self):pass
    class Server:
        def serve_forever(self,**kwargs):raise KeyboardInterrupt
        def server_close(self):calls.append('server')
    monkeypatch.setattr(module,'PreviewState',lambda _:SimpleNamespace())
    monkeypatch.setattr(module,'attach_payment_capture',lambda _:None)
    monkeypatch.setattr(module,'CompetitionServices',lambda *args:Resource('services'))
    monkeypatch.setattr(module,'RaceFlowDrive',lambda *args,**kwargs:Resource('application',True))
    monkeypatch.setattr(module,'make_server',lambda *args,**kwargs:Server())
    monkeypatch.setattr(module,'register_images',lambda *args,**kwargs:None)
    monkeypatch.setattr(pages,'add_page',lambda *args,**kwargs:None)
    monkeypatch.setattr(module,'PreviewWorker',Worker)
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    module.main('race',['--root',str(tmp_path)])
    assert calls==['application','services','fake-primary','fake-secondary','server']
