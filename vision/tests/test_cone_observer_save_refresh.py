"""A readonly scene save retires a camera batch already being decoded."""
from carvision.cone_observer import ConeObserverDrive
from test_cone_trial import CONFIG,Driver
import pytest


@pytest.mark.parametrize('old_batch_has_new_frame',[False,True])
def test_scene_save_cannot_publish_or_tick_the_old_inflight_camera_batch(tmp_path,monkeypatch,old_batch_has_new_frame):
    config=tmp_path/'cone-trial.json';config.write_bytes(CONFIG.read_bytes())
    drive=ConeObserverDrive(Driver(),{},config,tmp_path)
    drive.observing=True
    old_scene=drive.settings['scene']['surface']
    new_scene='indoor_white_line_track' if old_scene!='indoor_white_line_track' else 'red_running_track'
    readers=[];ticks=[]
    class Reader:
        def __init__(self,states,settings,clock,**kwargs):
            self.scene=settings['scene']['surface'];self.navigation={'scene':self.scene}
            self.navigator=object();self.preview=self.scene.encode();readers.append(self)
        def read(self):
            if len(readers)==1:
                drive.request('cone_settings',{'scene_surface':new_scene})
                assert drive.reset_reader
                if not old_batch_has_new_frame:return None
            else:drive.stop.set()
            return {'captured_s':drive.clock(),'scene':self.scene}
    monkeypatch.setattr('carvision.cone_observer.ConeCamera',Reader)
    monkeypatch.setattr(drive.run,'tick',lambda nav,now:ticks.append(dict(nav)))
    monkeypatch.setattr(drive.short,'tick',lambda *args:None)
    drive._work()
    assert len(readers)==2 and [reader.scene for reader in readers]==[old_scene,new_scene]
    assert ticks==[{'scene':new_scene}]
    assert drive.latest['scene']==new_scene and drive.preview==new_scene.encode()
    assert not drive.run.active and not drive.short.active and drive.run.actuator is None
