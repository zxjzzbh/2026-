"""The race reader may have a selected camera without a locked cone."""
from carvision.race_flow import load_bundle
from carvision.race_flow_camera import RaceFlowCamera
from test_cone_turn_controller import ROOT
from test_race_camera_efficiency import caches


def test_race_reader_clears_hint_when_current_camera_has_no_locked_target():
    reader=RaceFlowCamera(caches(),load_bundle(ROOT),ROOT)
    reader.set_avoidance_target('secondary',None)
    assert reader.navigator.avoidance_target is None
    reader.set_avoidance_target('secondary','secondary-A')
    assert reader.navigator.avoidance_target==('secondary','secondary-A')
    reader.set_avoidance_target('secondary',None)
    assert reader.navigator.avoidance_target is None
    reader.set_avoidance_target(None,None)
    assert reader.navigator.avoidance_target is None
