"""Source-aware observation assistance. Never supplies metric race inputs."""
import math

TARGETS = ('blue_board', 'crosswalk', 'traffic_light', 'cone', 'qr')


def source_cue(status, target, maximum_age_ms=600):
    result = status.get('result') if status else None
    age = status.get('host_frame_age_ms') if status else None
    fresh = (bool(status) and status.get('state') == 'ok' and isinstance(result, dict)
             and type(age) in (int, float) and math.isfinite(age) and 0 <= age <= maximum_age_ms)
    cue = {'fresh': fresh, 'presence': 'unknown', 'light': 'unknown', 'qr_values': [],
           'frame_id': result.get('frame_id') if isinstance(result, dict) else None,
           'host_frame_age_ms': age}
    if not fresh:
        return cue
    if target == 'qr':
        values = result.get('race', {}).get('qr_values', [])
        cue['qr_values'] = [value for value in values if isinstance(value, str) and value]
        cue['presence'] = 'present' if cue['qr_values'] else 'absent'
        return cue
    if result.get('detector_status') not in ('ok', 'experimental_color_shape'):
        return cue
    presence = result.get('presence', {}).get(target, 'unknown')
    cue['presence'] = presence if presence in ('present', 'absent') else 'unknown'
    if target == 'traffic_light' and cue['presence'] == 'present':
        light = result.get('traffic_light_state', 'unknown')
        cue['light'] = light if light in ('red', 'yellow', 'green') else 'unknown'
    return cue


def camera_assistance(primary, secondary, target='crosswalk'):
    if target not in TARGETS:
        raise ValueError('unknown camera assistance target')
    cues = {'primary': source_cue(primary, target), 'secondary': source_cue(secondary, target)}
    main, aux = cues['primary'], cues['secondary']
    selected, state, light = None, 'unknown', 'unknown'
    reason = 'source_or_detection_unknown'
    if target == 'traffic_light' and main['light'] != 'unknown' and aux['light'] != 'unknown' and main['light'] != aux['light']:
        reason = 'conflicting_traffic_light_observations'
    else:
        if main['presence'] == 'present':
            selected, state, reason = 'primary', 'present', 'primary_target_present'
            if target == 'traffic_light' and main['light'] == 'unknown' and aux['light'] != 'unknown':
                selected, state, reason = 'secondary', 'present', 'secondary_light_color_assistance'
        elif aux['presence'] == 'present':
            selected, state, reason = 'secondary', 'present', 'secondary_target_assistance'
        elif main['presence'] == aux['presence'] == 'absent':
            state, reason = 'absent', 'both_sources_target_not_detected'
        if selected is not None:
            light = cues[selected]['light']
    # Source of an observed cue and the view used to search are distinct.
    view = ('secondary' if selected == 'secondary' else
            'primary' if main['presence'] == 'present' or not aux['fresh'] else 'secondary')
    return {'mode': 'observation_assistance', 'hardware_output': False,
            'metric_fusion': False, 'autonomous_launch_authorized': False,
            'target': target, 'presence': state, 'traffic_light_state': light,
            'recognition_source': selected, 'recommended_view': view,
            'auxiliary_search': view == 'secondary' and selected is None,
            'reason': reason, 'sources': cues,
            'qr_values': [] if selected is None else cues[selected]['qr_values']}
