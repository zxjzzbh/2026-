"""Explicit synthetic observation contracts, never physical pass records."""
from test_cone_turn_controller import lane


def ground_nav(t,identifier='secondary-A',*,side='left',y=.55,fid=None):
    reference=.3 if side=='left' else .7
    fraction=.7 if side=='left' else .3
    image_reference=.1+.8*reference
    x=.1+.8*fraction
    fid=round(t*1000) if fid is None else fid
    proof={'target_id':identifier,'camera':'secondary','frame_id':fid,
           'captured_s':t,'within_observed_band':True,
           'reference_fraction_at_foot':reference,'cone_fraction_at_foot':fraction,
           'cone_half_width_fraction':.03,'foot_y_fraction':y}
    target={'id':identifier,'fraction':fraction,'half_width':.03,'confirmed':True,
            'near':y>=.6,'exit_band':y>=.7,'y_fraction':y,
            'image_fraction':x,'half_image_width':.02,
            'bbox_xyxy':[(x-.02)*480,(y-.06)*360,(x+.02)*480,min(360,(y+.01)*360)],
            'pass_evidence':proof,'identity_revision':0}
    visual={'id':identifier,'current_pixel':True,'confirmed':True,
            'captured_s':t,'frame_id':proof['frame_id'],'source_camera':'secondary',
            'image_fraction':x,'half_image_width':.02,'image_reference_fraction':image_reference,
            'y_fraction':y,'bbox_xyxy':target['bbox_xyxy'],'identity_revision':0}
    return lane(t,fid=fid,near=reference,far=reference,cones=[target],visual_cones=[visual],
                obstacle_diagnostics=[{'id':identifier,'reason':'current_observed_lane_position'}])


def exit_nav(t,identifier='secondary-A',*,side='left',index=0,fid=None):
    nav=ground_nav(t,identifier,side=side,y=(.86,.90,.94)[index],fid=fid)
    # Ground is out of the actually observed band; current pixels alone are
    # exit evidence for an already established candidate, never new gates.
    target=nav['cones'][0];target.pop('pass_evidence')
    target.update(near_field_continuation=True,position_source='last_valid_ground_observation')
    pixel=nav['visual_cones'][0]
    pixel['image_fraction']=(.82,.86,.90)[index] if side=='left' else (.18,.14,.10)[index]
    x,y=pixel['image_fraction'],pixel['y_fraction']
    pixel['bbox_xyxy']=[(x-.02)*480,(y-.06)*360,(x+.02)*480,min(360,(y+.01)*360)]
    return nav


def estimate_one(controller,start,identifier='secondary-A',*,side='left'):
    result=None
    for i,y in enumerate((.55,.63,.71,.73,.76)):
        t=round(start+i*.1,8);result=controller.step(ground_nav(t,identifier,side=side,y=y),t)
    for i in range(3):
        t=round(start+.5+i*.1,8);result=controller.step(exit_nav(t,identifier,side=side,index=i),t)
    return t,result
