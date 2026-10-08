from copy import deepcopy

import pytest

from carvision.camera_assistance import camera_assistance


def status(presence='absent', light='unknown', age=100, detector='experimental_color_shape'):
    return {'state': 'ok', 'host_frame_age_ms': age,
            'result': {'frame_id': 10, 'detector_status': detector,
                       'presence': {label: presence for label in ('blue_board', 'crosswalk', 'traffic_light', 'cone')},
                       'traffic_light_state': light, 'race': {'qr_values': []}}}


def test_main_priority_aux_search_and_return_use_source_provenance():
    main, aux = status('present'), status('present')
    before = deepcopy((main, aux))
    result = camera_assistance(main, aux)
    assert result['recognition_source'] == result['recommended_view'] == 'primary'
    assert result['hardware_output'] is False and result['metric_fusion'] is False
    assert (main, aux) == before
    result = camera_assistance(status(), aux)
    assert result['presence'] == 'present' and result['recognition_source'] == 'secondary'
    assert result['recommended_view'] == 'secondary'
    result = camera_assistance(status('present'), aux)
    assert result['recognition_source'] == result['recommended_view'] == 'primary'


def test_search_view_does_not_invent_a_detection_when_both_sources_miss():
    result = camera_assistance(status(), status())
    assert result['recognition_source'] is None and result['presence'] == 'absent'
    assert result['recommended_view'] == 'secondary' and result['auxiliary_search'] is True
    assert result['autonomous_launch_authorized'] is False


@pytest.mark.parametrize('age', [601, float('nan'), float('inf'), None, True, -1])
def test_stale_or_invalid_auxiliary_image_cannot_override_main(age):
    result = camera_assistance(status(), status('present', age=age))
    assert result['recognition_source'] is None and result['presence'] == 'unknown'
    assert result['recommended_view'] == 'primary'


def test_disabled_or_failed_camera_does_not_become_absence():
    result = camera_assistance(status(detector='disabled'), status())
    assert result['presence'] == 'unknown'
    failed = status('present')
    failed['state'] = 'error'
    assert camera_assistance(status(), failed)['presence'] == 'unknown'


@pytest.mark.parametrize('main_light,aux_light', [('red', 'green'), ('yellow', 'green'), ('red', 'yellow')])
def test_conflicting_lights_never_authorize_green(main_light, aux_light):
    result = camera_assistance(status('present', main_light), status('present', aux_light), 'traffic_light')
    assert result['presence'] == result['traffic_light_state'] == 'unknown'
    assert result['recognition_source'] is None
    assert result['reason'] == 'conflicting_traffic_light_observations'


def test_auxiliary_can_read_color_when_main_only_sees_lamp_shape():
    result = camera_assistance(status('present'), status('present', 'red'), 'traffic_light')
    assert result['recognition_source'] == 'secondary' and result['traffic_light_state'] == 'red'
    assert result['recommended_view'] == 'secondary'
    assert camera_assistance(status('present', 'red'), status('present', 'red'), 'traffic_light')['recommended_view'] == 'primary'


def test_qr_assistance_only_reports_text_and_does_not_transfer_money():
    aux = status(detector='disabled')
    aux['result']['race']['qr_values'] = ['test payment text']
    result = camera_assistance(status(), aux, 'qr')
    assert result['recognition_source'] == 'secondary'
    assert result['qr_values'] == ['test payment text']
    assert result['hardware_output'] is False


def test_unknown_target_is_rejected():
    with pytest.raises(ValueError):
        camera_assistance(status(), status(), 'parking')
