"""Keep synthetic flow observations compatible with this car's active profile.

The historical test scenarios use .5 as a far sighting and .75 as the stop row.
Map those synthetic cases onto the current measured camera reference. This
changes test observations only; it never changes or verifies a field profile.
"""
from carvision.crosswalk_calibration import reference_y


def current_crosswalk_sample(controller, sample):
    profile=controller.b['brake'].get('crosswalk_calibration')
    if profile is None:return sample
    result=dict(sample)
    result.update(source_camera='secondary',calibration_id=profile['profile_id'],
                  image_feature=profile['image_feature'],near_limit_reached=False,
                  source_changed=False,near_edge_y_normalized=None)
    if sample.get('candidate') is True:
        far=reference_y(profile,'secondary',profile['overlap_range_m'][1])['near_edge_y_normalized']
        stop=reference_y(profile,'secondary',controller.b['brake']['crosswalk_distance_trigger_m'])['near_edge_y_normalized']
        # At the user's maximum (65cm) trigger, the farthest measured row
        # is already the stop row. A synthetic first sighting must then lie
        # beyond that range, with presence only and no extrapolated distance.
        if far>=stop:far=max(1e-6,stop-.02)
        fraction=max(0.,min(1.,(sample['far_edge_y_normalized']-.5)/.25))
        result['near_edge_y_normalized']=far+(stop-far)*fraction
    return result
