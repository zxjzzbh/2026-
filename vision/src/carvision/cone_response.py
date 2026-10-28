"""Small image-space policies; none of these quantities verifies wheel motion."""
from .brake_parking import numeric


def combine_guidance(lateral,heading,avoiding):
    """Keep opposing heading feedback from erasing a live avoidance demand."""
    correction=.65*heading
    if avoiding and lateral*correction<0:
        correction=max(-.25*abs(lateral),min(.25*abs(lateral),correction))
    return lateral+correction,correction


def current_visual(nav,identifier):
    """A same-frame measured pixel target can extend an existing lock only."""
    for entry in nav.get('visual_cones',[]):
        if (entry.get('id')!=identifier or entry.get('current_pixel') is not True
                or entry.get('confirmed') is not True):continue
        if (any(key not in entry for key in ('captured_s','frame_id','source_camera'))
                or entry['captured_s']!=nav.get('captured_s')
                or entry['frame_id']!=nav.get('frame_id')
                or entry['source_camera']!=nav.get('source_camera')):continue
        x,half,reference=(entry.get(k) for k in ('image_fraction','half_image_width','image_reference_fraction'))
        if (all(numeric(v) for v in (x,half,reference)) and 0<=x<=1
                and 0<half<=.5 and 0<=reference<=1):return entry
    return None


def visual_goal(entry,side):
    x=entry['image_fraction'];reference=entry['image_reference_fraction']
    side=side or ('left' if x>=reference else 'right')
    goal=max(.02,min(.98,x+(-1 if side=='left' else 1)*(entry['half_image_width']+.15)))
    return goal,reference,side


def blocking_replacement(cones,identifier,old_target,goals,half):
    """Prioritize a currently confirmed blocker over old tail bookkeeping."""
    for candidate in cones:
        if (candidate['id']==identifier or candidate.get('near_field_continuation')
                or candidate.get('provisional') or candidate.get('confirmed') is False
                or not numeric(candidate.get('fraction')) or not numeric(candidate.get('half_width'))):continue
        # A farther cone must not steal a still-visible foreground target.
        if (old_target and not old_target.get('near_field_continuation')
                and candidate.get('y_fraction',0)<old_target.get('y_fraction',0)+.03):continue
        clearance=half+candidate['half_width']+.03
        if any(numeric(goal) and abs(candidate['fraction']-goal)<=clearance for goal in goals):
            return candidate
    return None
