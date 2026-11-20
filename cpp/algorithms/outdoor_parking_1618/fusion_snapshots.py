"""Image/metadata logging only; no GPIO, camera or controller changes."""
from datetime import datetime
import json
from pathlib import Path
import cv2


def transition_events(previous, current):
    previous = previous or {}
    tags = [] if previous else ['RUN_START']
    if current.get('boxes') and not previous.get('boxes'):
        tags.append('CONE_SEEN')
    target = current.get('cone_target_id')
    if target is not None and target != previous.get('cone_target_id'):
        tags.append('TARGET_LOCKED')
    phase = current.get('cone_state')
    if phase != previous.get('cone_state'):
        tag = {'AVOID': 'AVOID_START', 'PASS': 'PASS_START', 'ALIGN': 'ALIGN_START'}.get(phase)
        if tag:
            tags.append(tag)
    if current.get('cone_pass_confirmed') and not previous.get('cone_pass_confirmed'):
        tags.append('PASS_CONFIRMED')
    if (phase == 'STRAIGHT' and current.get('cone_reason') == 'RESUME_STRAIGHT'
            and (previous.get('cone_state') != 'STRAIGHT'
                 or previous.get('cone_reason') != 'RESUME_STRAIGHT')):
        tags.append('AVOID_COMPLETE')
    if current.get('state') != previous.get('state'):
        tags.append(current.get('state') or 'STATE_CHANGE')
    return list(dict.fromkeys(tags))


def save_snapshot(folder, raw, debug, event, tags, drive, suffix='', final_status=None):
    folder = Path(folder)
    prefix = '%06d_%s%s' % (event['frame'], event.get('state', 'UNKNOWN'), '_' + suffix if suffix else '')
    common = dict(event, source='fusion', run=folder.name,
                  mode='DRIVE' if drive else 'OBSERVE', events=tags or ['PERIODIC'],
                  saved_at=datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                  frame_kind='last_available' if suffix else 'state_frame')
    if final_status is not None:
        common['final_status'] = final_status
    for kind, frame in [('raw', raw), ('debug', debug)]:
        if frame is None:
            continue
        dest = folder / (prefix + '_' + kind + '.jpg')
        tmp = dest.with_name(dest.stem + '.partial.jpg')
        if not cv2.imwrite(str(tmp), frame):
            raise OSError('Snapshot write failed: ' + str(dest))
        tmp.replace(dest)
        sidecar = dest.with_suffix('.json')
        pending = sidecar.with_suffix('.json.partial')
        pending.write_text(json.dumps(dict(common, image_kind=kind), ensure_ascii=False), encoding='utf-8')
        pending.replace(sidecar)
