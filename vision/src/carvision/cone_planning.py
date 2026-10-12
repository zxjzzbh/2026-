"""Measured-coordinate route suggestions; no PWM or distance assumptions.

Paths use rear-axle coordinates (x right, y forward). Each sample checks an
oriented rectangle covering the entire vehicle against every cone, not just
a single lateral target. This is a planning component, not field validation.
"""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .autonomy_profile import GroundProjection
from .brake_parking import numeric
from .cone_slalom import SlalomProgress


def load_settings(path):
    s=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    return validate_settings(s)


def validate_settings(s):
    if not isinstance(s,dict):raise ValueError('settings must be a JSON object')
    if s.get('schema_version')!=1 or s.get('camera')!='secondary' or s.get('aux_camera')!='primary' or s.get('image_size')!=[480,360]:
        raise ValueError('cone trial requires existing secondary/primary 480x360 streams')
    if s.get('target_count')!=2:raise ValueError('this contest stage has two cones')
    for k in ('blue_hsv_lower','blue_hsv_upper'):
        if not isinstance(s.get(k),list) or len(s[k])!=3 or any(type(v) is not int for v in s[k]):raise ValueError(k)
    if not (0<=s['blue_hsv_lower'][0]<=s['blue_hsv_upper'][0]<=179
            and all(0<=s['blue_hsv_lower'][i]<=s['blue_hsv_upper'][i]<=255 for i in (1,2))):raise ValueError('invalid HSV range')
    if type(s.get('confirm_frames')) is not int or not 2<=s['confirm_frames']<=8:raise ValueError('confirm_frames')
    for k,lo,hi in [('confirm_s',.05,.5),('track_timeout_s',.2,1),('max_frame_age_s',.1,.25)]:
        if not numeric(s.get(k)) or not lo<=s[k]<=hi:raise ValueError(k)
    for section in ('geometry','ground','motion'):
        if not isinstance(s.get(section),dict):raise ValueError(section)
    scene=s.get('scene',{})
    if not isinstance(scene,dict):raise ValueError('scene')
    if type(scene.get('white_v_min',200)) is not int or not 0<=scene.get('white_v_min',200)<=255:
        raise ValueError('白线亮度阈值必须为 0–255 的整数')
    if scene.get('surface','red_running_track') not in ('red_running_track','indoor_white_line_track'):
        raise ValueError('unsupported cone track surface')
    if scene.get('boundary_policy','whole_vehicle_inside_white_inner_edges')!='whole_vehicle_inside_white_inner_edges':
        raise ValueError('vehicle must stay inside white inner edges')
    if not numeric(scene.get('red_ground_min_fraction',.15)) or not 0<scene.get('red_ground_min_fraction',.15)<=1:
        raise ValueError('red_ground_min_fraction')
    return s


def readiness(s):
    missing=[];g=s['geometry'];m=s['motion'];ground=s['ground']
    for k in ('lane_width_m','vehicle_width_m','vehicle_length_m','rear_axle_to_front_m','rear_overhang_m',
              'cone_height_m','cone_base_width_m','clearance_m'):
        if not numeric(g.get(k)) or g[k]<=0:missing.append('geometry.'+k)
    if g.get('measured') is not True:missing.append('geometry.measured')
    if not missing:
        if g['lane_width_m']<=g['vehicle_width_m']+2*g['clearance_m']:missing.append('geometry.lane_too_narrow')
        if abs(g['rear_axle_to_front_m']+g['rear_overhang_m']-g['vehicle_length_m'])>.02:missing.append('geometry.length_inconsistent')
    try:
        if ground.get('coordinate_frame')!='rear_axle_x_right_y_forward':raise ValueError('frame')
        if ground.get('image_size')!=s['image_size']:raise ValueError('size')
        GroundProjection(ground)
    except (ValueError,TypeError,KeyError):missing.append('ground.verified_mapping')
    if not numeric(ground.get('max_error_m')) or not 0<=ground['max_error_m']<=.1:missing.append('ground.max_error_m')
    for k in ('minimum_turn_radius_m','maximum_tracking_error_m','braking_distance_m'):
        if not numeric(m.get(k)) or m[k]<=0:missing.append('motion.'+k)
    for k in ('verified','pose_feedback_verified'):
        if m.get(k) is not True:missing.append('motion.'+k)
    return {'metric_planning_ready':not missing,'missing':missing,'hardware_output':False,
            'field_collision_clearance_verified':False}


def project_tracks(tracks,settings):
    """Only use the designated view. Auxiliary detections are never double-counted."""
    projection=GroundProjection(settings['ground'])
    metric=[];unresolved=[]
    for t in tracks:
        if not t['confirmed'] or not t['visible'] or not t['ground_contact_usable']:
            unresolved.append(t['id']);continue
        if t.get('boundary_relation')!='inside':
            unresolved.append(t['id']);continue
        try:x,y=projection.point(t['foot_pixel'])
        except ValueError:unresolved.append(t['id']);continue
        metric.append({'id':t['id'],'x_m':x,'y_m':y,'radius_m':settings['geometry']['cone_base_width_m']/2})
    return metric,unresolved


def smooth_path(knots,step=.02,road_slope=0.):
    # Quintic segments provide zero slope/curvature at the explicitly selected
    # clearance stations. Sharp/too-short changes are rejected by curvature.
    rows=[]
    for index,((y0,x0),(y1,x1)) in enumerate(zip(knots,knots[1:])):
        if y1<=y0:raise ValueError('path stations must increase')
        span=y1-y0
        m0=0. if index==0 else road_slope;m1=road_slope;delta=x1-x0
        a1=m0*span;a3=10*delta-6*m0*span-4*m1*span
        a4=-15*delta+8*m0*span+7*m1*span;a5=6*delta-3*m0*span-3*m1*span
        for u in np.linspace(0,1,max(3,math.ceil(span/step)+1))[:-1]:
            x=x0+a1*u+a3*u**3+a4*u**4+a5*u**5
            slope=(a1+3*a3*u*u+4*a4*u**3+5*a5*u**4)/span
            curve=(6*a3*u+12*a4*u*u+20*a5*u**3)/(span*span)
            rows.append({'x_m':float(x),'y_m':float(y0+span*u),
                         'yaw_rad':float(math.atan(slope)),'curvature':float(curve/(1+slope*slope)**1.5)})
    rows.append({'x_m':knots[-1][1],'y_m':knots[-1][0],'yaw_rad':math.atan(road_slope),'curvature':0.})
    return rows


def distance_to_path(path,x,y,start_index=0):
    """Distance to the sampled polyline, with monotone segment progress.

    Distance to vertices alone includes sampling error even on the route.
    Clamp projections to segments so points beyond an endpoint still fail.
    """
    if not path or not 0<=start_index<len(path):raise ValueError('empty or invalid reference path')
    last=path[-1]
    best=(math.hypot(x-last['x_m'],y-last['y_m']),len(path)-1)
    for i in range(start_index,len(path)-1):
        a,b=path[i],path[i+1]
        dx,dy=b['x_m']-a['x_m'],b['y_m']-a['y_m'];length2=dx*dx+dy*dy
        fraction=min(1.,max(0.,((x-a['x_m'])*dx+(y-a['y_m'])*dy)/length2)) if length2 else 0.
        error=math.hypot(x-a['x_m']-fraction*dx,y-a['y_m']-fraction*dy)
        if error<best[0]:best=(error,i+1 if fraction==1. else i)
    return best


def path_clear(path,cones,settings,lane_bounds,lane_heading_rad=0.):
    g=settings['geometry'];m=settings['motion']
    # Include localization/tracking uncertainty and half a sampling step so
    # discrete checks are not presented as a zero-margin continuous proof.
    pad=g['clearance_m']+settings['ground']['max_error_m']+m['maximum_tracking_error_m']+.02
    half=g['vehicle_width_m']/2+pad;front=g['rear_axle_to_front_m']+pad;rear=g['rear_overhang_m']+pad
    left,right=lane_bounds
    road_co,road_si=math.cos(lane_heading_rad),math.sin(lane_heading_rad)
    driving=settings.get('driving',{})
    radii=[driving.get(k) for k in ('steering_radius_1570_cm','steering_radius_1650_cm')]
    limits=sorted(100/r for r in radii) if all(numeric(r) and r!=0 for r in radii) else None
    for p in path:
        if abs(p['curvature'])>1/m['minimum_turn_radius_m']:return False
        if limits and not limits[0]<=p['curvature']<=limits[1]:return False
        co,si=math.cos(p['yaw_rad']),math.sin(p['yaw_rad'])
        corners=[road_co*(p['x_m']+co*x+si*y)-road_si*(p['y_m']-si*x+co*y)
                 for x in (-half,half) for y in (-rear,front)]
        if min(corners)<left or max(corners)>right:return False
        for obstacle in cones:
            dx,dy=obstacle['x_m']-p['x_m'],obstacle['y_m']-p['y_m']
            bx,by=co*dx-si*dy,si*dx+co*dy
            # Circle versus the padded oriented vehicle rectangle.
            closest_x=min(half,max(-half,bx));closest_y=min(front,max(-rear,by))
            if math.hypot(bx-closest_x,by-closest_y)<=obstacle['radius_m']:return False
    return True


class ConeRoutePlanner:
    def __init__(self,settings):
        self.s=copy.deepcopy(settings);self.side_lock=None

    def plan(self,cones,*,lane_bounds,lane_fresh=True,coverage_forward_m=None,lane_heading_rad=0.,return_to_center=True,required_sides=None):
        report=readiness(self.s)
        stop={'action':'stop','path':[],'hardware_output':False,'field_collision_clearance_verified':False}
        if not report['metric_planning_ready']:return {**stop,'reason':'needs_measured_configuration','missing':report['missing']}
        if (not lane_fresh or not isinstance(lane_bounds,(tuple,list)) or len(lane_bounds)!=2
                or not all(numeric(v) for v in lane_bounds) or lane_bounds[0]>=lane_bounds[1]):
            return {**stop,'reason':'lane_boundaries_unknown'}
        g=self.s['geometry'];m=self.s['motion']
        if not numeric(lane_heading_rad) or abs(lane_heading_rad)>.4:return {**stop,'reason':'invalid_lane_heading'}
        road_co,road_si=math.cos(lane_heading_rad),math.sin(lane_heading_rad)
        def path_x(normal,y):return (normal+road_si*y)/road_co
        if not numeric(coverage_forward_m) or coverage_forward_m<=g['rear_axle_to_front_m']+m['braking_distance_m']:
            return {**stop,'reason':'insufficient_observed_forward_region'}
        if len(cones)>2:return {**stop,'reason':'more_than_two_or_duplicate_obstacles'}
        for c in cones:
            if not isinstance(c,dict) or not isinstance(c.get('id'),str) or any(not numeric(c.get(k)) for k in ('x_m','y_m','radius_m')) or c['radius_m']<=0:
                return {**stop,'reason':'invalid_obstacle_geometry'}
        if len({c['id'] for c in cones})!=len(cones):return {**stop,'reason':'duplicate_obstacle_id'}
        if required_sides is not None and (not isinstance(required_sides,dict)
                or any(required_sides.get(c['id']) not in ('left','right') for c in cones)):
            return {**stop,'reason':'invalid_required_cone_sides'}
        horizon=coverage_forward_m-g['rear_axle_to_front_m']-m['braking_distance_m']
        obstacles=sorted(cones,key=lambda c:c['y_m'])
        margin=g['clearance_m']+self.s['ground']['max_error_m']+m['maximum_tracking_error_m']+.025
        needed=g['vehicle_width_m']/2+margin
        left,right=lane_bounds
        if right-left>g['lane_width_m']+2*self.s['ground']['max_error_m']:
            return {**stop,'reason':'observed_corridor_exceeds_measured_lane_width'}
        if left+needed>=right-needed:return {**stop,'reason':'lane_too_narrow'}
        # Build candidate offset stations for both obstacles. Keep only short
        # lists to remain practical on the Pi, then validate the full path.
        options=[(0.,[(0.,0.)])]
        for c in obstacles:
            if c['y_m']<0:continue
            enter=c['y_m']-g['rear_axle_to_front_m']-margin-c['radius_m']
            leave=c['y_m']+g['rear_overhang_m']+margin+c['radius_m']
            if leave>=horizon:return {**stop,'reason':'cone_exit_outside_observed_region'}
            next_options=[]
            for offset,knots in options:
                normal=road_co*c['x_m']-road_si*c['y_m']
                reserve=m['maximum_tracking_error_m']+.02 if required_sides is not None else 0.
                targets=[offset,normal-c['radius_m']-needed-reserve,normal+c['radius_m']+needed+reserve]
                for target in targets:
                    if required_sides is not None:
                        sign=-1 if required_sides[c['id']]=='left' else 1
                        if sign*(target-normal)<c['radius_m']+needed-.005-1e-9:continue
                    if not left+needed<=target<=right-needed:continue
                    if enter<=knots[-1][0]:
                        # Very close cones must share the current side; no
                        # instantaneous weave between overlapping stations.
                        if abs(target-offset)>.001 or leave<=knots[-1][0]:continue
                        new=knots+[(leave,path_x(target,leave))]
                    else:new=knots+[(enter,path_x(target,enter)),(leave,path_x(target,leave))]
                    next_options.append((target,new))
            options=next_options
        candidates=[]
        for offset,knots in options:
            if knots[-1][0]>=horizon:continue
            # Try returning to centre only when the remaining geometry permits
            # it; otherwise hold the side and replan as new road is observed.
            for end in ((left+right)/2,offset) if return_to_center else (offset,):
                path=smooth_path(knots+[(horizon,path_x(end,horizon))],road_slope=math.tan(lane_heading_rad))
                # The existing white-line observer requires a near-forward road
                # view. Do not offer a slalom that would immediately lose it.
                if required_sides is not None and any(abs(p['yaw_rad']-lane_heading_rad)>.4 for p in path):continue
                if not path_clear(path,obstacles,self.s,lane_bounds,lane_heading_rad):continue
                normals=[road_co*p['x_m']-road_si*p['y_m'] for p in path]
                side='right' if max(normals)>abs(min(normals)) else 'left'
                if all(abs(x)<.001 for x in normals):side='center'
                cost=sum(abs(p['curvature'])*.03+abs(x)*.01 for p,x in zip(path,normals))
                cost+=5*abs(end-(left+right)/2)
                if self.side_lock and side not in (self.side_lock,'center'):cost+=10
                candidates.append((cost,side,path))
        if not candidates:return {**stop,'reason':'no_swept_collision_clear_route'}
        cost,side,path=min(candidates,key=lambda x:x[0])
        if obstacles:self.side_lock=side
        else:self.side_lock=None
        return {'action':'follow_candidate_path','reason':'all_observed_cones_checked','side':side,'path':path,
                'cost':cost,'required_sides':dict(required_sides or {}),
                'hardware_output':False,'field_collision_clearance_verified':False}


class ConePassCounter:
    """Passed IDs require world pose, never an image-disappearance event."""
    def __init__(self,settings):
        self.s=settings;self.world={};self.passed=set();self.seen_ahead=set();self.aliases={}

    def update(self,observed_world,pose):
        if (not readiness(self.s)['metric_planning_ready'] or pose.get('valid') is not True
                or any(not numeric(pose.get(k)) for k in ('x_m','y_m','yaw_rad','error_bound_m'))
                or not 0<=pose['error_bound_m']<=self.s['motion']['maximum_tracking_error_m']):
            return {'passed_ids':sorted(self.passed),'complete':False,'reason':'pose_unavailable'}
        co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        for cone in observed_world:
            if (not isinstance(cone.get('id'),str) or any(not numeric(cone.get(k)) for k in ('x_m','y_m','radius_m'))
                    or cone['radius_m']<=0):continue
            key=self.aliases.get(cone['id'],cone['id'])
            if key not in self.world:
                # A re-acquired image ID at the same fixed world position must
                # not count the same physical cone twice. Ambiguous nearby
                # identities are conservatively merged, not counted as two.
                for old,prior in self.world.items():
                    if math.hypot(prior['x_m']-cone['x_m'],prior['y_m']-cone['y_m'])<max(prior['radius_m'],cone['radius_m'])+self.s['ground']['max_error_m']:
                        key=old;break
            self.aliases[cone['id']]=key
            if key in self.passed:continue
            self.world[key]={**cone,'id':key}
            forward=si*(cone['x_m']-pose['x_m'])+co*(cone['y_m']-pose['y_m'])
            if forward>self.s['geometry']['rear_axle_to_front_m']+cone['radius_m']:
                self.seen_ahead.add(key)
        rear=self.s['geometry']['rear_overhang_m'];margin=self.s['geometry']['clearance_m']+pose['error_bound_m']
        for key,c in self.world.items():
            dx,dy=c['x_m']-pose['x_m'],c['y_m']-pose['y_m']
            behind=si*dx+co*dy
            if key in self.seen_ahead and behind+c['radius_m']+margin < -rear:self.passed.add(key)
        if len(self.world)>2:
            return {'passed_ids':sorted(self.passed),'complete':False,'reason':'ambiguous_obstacle_count'}
        return {'passed_ids':sorted(self.passed),'complete':len(self.passed)>=2,'reason':'pose_based_pass_check'}


class ConeMission:
    """Two ordered opposite-side passes, then lane recovery. No hardware I/O."""
    def __init__(self,settings):
        self.s=settings;self.planner=ConeRoutePlanner(settings);self.counter=ConePassCounter(settings)
        self.phase='waiting_for_measurements';self.last_id=self.last_t=None
        self.slalom=None
        self.reference=[];self.reference_index=0
        self.reference_returns_to_lane=False

    def _progress(self):
        return self.slalom.status() if self.slalom else {'required':True,'order':[],
            'passed_ids':[],'gate_verified_ids':[],'complete':False,'fault':None}

    def _set_reference(self,path,pose,arguments):
        co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        self.reference=[{**p,'x_m':pose['x_m']+co*p['x_m']+si*p['y_m'],
            'y_m':pose['y_m']-si*p['x_m']+co*p['y_m'],'yaw_rad':p['yaw_rad']+pose['yaw_rad']}
            for p in path]
        self.reference_index=0
        h=arguments['lane_heading_rad'];end=path[-1]
        self.reference_returns_to_lane=abs(math.cos(h)*end['x_m']-math.sin(h)*end['y_m']-sum(arguments['lane_bounds'])/2)<.001

    def _follow_reference(self,pose,cones,arguments):
        stop={'action':'stop','path':[],'hardware_output':False,'field_collision_clearance_verified':False}
        if not arguments['lane_fresh']:return {**stop,'reason':'lane_boundaries_unknown'}
        bounds=arguments['lane_bounds']
        if (not isinstance(bounds,(tuple,list)) or len(bounds)!=2 or not all(numeric(v) for v in bounds)
                or bounds[0]>=bounds[1]
                or bounds[1]-bounds[0]>self.s['geometry']['lane_width_m']+2*self.s['ground']['max_error_m']):
            return {**stop,'reason':'lane_boundaries_unknown'}
        lookahead=self.s.get('driving',{}).get('lookahead_m',.4)
        end=self.reference[-1]
        if (self._progress()['complete'] and self.reference_returns_to_lane
                and math.hypot(end['x_m']-pose['x_m'],end['y_m']-pose['y_m'])<lookahead+.2):
            # Supply a straight, centred lookahead after the finite recovery
            # curve. These points are still clipped to freshly observed road
            # and collision checked below; the controller never chases a point
            # immediately behind the vehicle at the original path endpoint.
            for distance in np.arange(.02,1.01,.02):
                self.reference.append({'x_m':end['x_m']+math.sin(self.slalom.heading)*distance,
                    'y_m':end['y_m']+math.cos(self.slalom.heading)*distance,
                    'yaw_rad':self.slalom.heading,'curvature':0.})
        # Track the chosen world path instead of rebuilding a tighter turn from
        # rest at every frame. The two-cone side assignment is never reselected.
        error,index=distance_to_path(self.reference,pose['x_m'],pose['y_m'],self.reference_index)
        self.reference_index=index
        if error>self.s['motion']['maximum_tracking_error_m']:
            return {**stop,'reason':'slalom_reference_tracking_error'}
        coverage=arguments['coverage_forward_m'];heading=arguments['lane_heading_rad']
        if not numeric(coverage) or not numeric(heading) or abs(heading)>.4:
            return {**stop,'reason':'slalom_reference_view_unavailable'}
        horizon=coverage-self.s['geometry']['rear_axle_to_front_m']-self.s['motion']['braking_distance_m']
        path=[];co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        for point in self.reference[index:]:
            dx,dy=point['x_m']-pose['x_m'],point['y_m']-pose['y_m']
            x,y=co*dx-si*dy,si*dx+co*dy
            if 0<y<=horizon:path.append({**point,'x_m':x,'y_m':y,'yaw_rad':point['yaw_rad']-pose['yaw_rad']})
        if not path or path[-1]['y_m']<lookahead:
            return {**stop,'reason':'slalom_reference_view_unavailable'}
        current={'x_m':0.,'y_m':0.,'yaw_rad':0.,'curvature':0.}
        if not path_clear([current]+path,cones,self.s,arguments['lane_bounds'],heading):
            return {**stop,'reason':'slalom_reference_blocked'}
        return {'action':'follow_candidate_path','reason':'following_locked_alternating_route',
                'path':path,'required_sides':dict(self.slalom.sides),
                'hardware_output':False,'field_collision_clearance_verified':False}

    def step(self,sample,now):
        stop={'action':'stop','hardware_output':False,'phase':self.phase,
              'passed_ids':self._progress()['passed_ids'],'slalom':self._progress()}
        ready=readiness(self.s)
        if not ready['metric_planning_ready']:return {**stop,'reason':'needs_measured_configuration','missing':ready['missing']}
        fid,t=sample.get('frame_id'),sample.get('captured_s')
        if (not numeric(now) or not numeric(t) or not 0<=now-t<=self.s['max_frame_age_s'] or type(fid) is not int
                or self.last_id is not None and fid<=self.last_id or self.last_t is not None and t<=self.last_t):
            self.phase='fault';return {**stop,'phase':self.phase,'reason':'stale_or_repeated_observation'}
        self.last_id,self.last_t=fid,t
        if self.phase in ('fault','complete'):return {**stop,'reason':'terminal_state'}
        pose=sample.get('pose') or {}
        passed=self.counter.update(sample.get('observed_world',[]),pose)
        if passed['reason']!='pose_based_pass_check':
            self.phase='waiting_for_feedback';return {**stop,'phase':self.phase,'reason':'pose_or_obstacle_uncertain'}
        if self.slalom:
            self.slalom.update(pose,t,self.counter.world)
            progress=self._progress()
            stop.update(slalom=progress,passed_ids=progress['passed_ids'])
            if progress['fault']:
                self.phase='fault';return {**stop,'phase':self.phase,'reason':progress['fault']}
        # A fresh measured pose remains useful while a blue candidate is being
        # confirmed. Keep continuity/jump checks alive during the neutral pause;
        # unknown obstacles still prevent all forward and completion decisions.
        if sample.get('unresolved_obstacles'):
            self.phase='waiting_for_feedback';return {**stop,'phase':self.phase,'reason':'pose_or_obstacle_uncertain'}
        if self._progress()['complete']:
            offset=sample.get('lane_offset_m')
            heading=sample.get('lane_heading_rad')
            if (sample.get('lane_valid') is True and numeric(offset) and numeric(heading)
                    and abs(offset)<=self.s['motion']['maximum_tracking_error_m'] and abs(heading)<=.05):
                self.phase='complete';return {**stop,'phase':self.phase,'reason':'alternating_cones_passed_and_lane_recovered'}
            self.phase='return_to_lane'
        cones=[];co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        anchors={c['id']:c for c in self.slalom.cones} if self.slalom else self.counter.world
        for key,c in anchors.items():
            if key in self._progress()['passed_ids']:continue
            dx,dy=c['x_m']-pose['x_m'],c['y_m']-pose['y_m']
            cones.append({**c,'x_m':co*dx-si*dy,'y_m':si*dx+co*dy})
        arguments=dict(lane_bounds=sample.get('lane_bounds_m'),lane_fresh=sample.get('lane_valid') is True,
            coverage_forward_m=sample.get('coverage_forward_m'),lane_heading_rad=sample.get('lane_heading_rad',0.))
        if self.slalom is None:
            if len(cones)!=2 or any(c['id'] not in self.counter.seen_ahead or c['y_m']<=0 for c in cones):
                self.phase='waiting_for_two_cones'
                return {**stop,'phase':self.phase,'reason':'need_two_confirmed_cones_ahead'}
            ordered=sorted(cones,key=lambda c:c['y_m'])
            g=self.s['geometry'];m=self.s['motion']
            margin=g['clearance_m']+self.s['ground']['max_error_m']+m['maximum_tracking_error_m']+.025
            occupied=g['vehicle_length_m']+sum(c['radius_m'] for c in ordered)+2*margin
            if ordered[1]['y_m']-ordered[0]['y_m']<=occupied:
                self.phase='waiting_for_clear_route'
                return {**stop,'phase':self.phase,'reason':'cones_too_close_for_alternation'}
            candidates=[]
            for sides in [('left','right'),('right','left')]:
                required=dict(zip([c['id'] for c in ordered],sides))
                candidate=ConeRoutePlanner(self.s).plan(cones,**arguments,required_sides=required)
                if candidate['action']=='follow_candidate_path':candidates.append(candidate)
            if not candidates:
                self.phase='waiting_for_clear_route'
                return {**stop,'phase':self.phase,'reason':'no_feasible_alternating_route'}
            result=min(candidates,key=lambda r:r['cost'])
            self.slalom=SlalomProgress(self.s,[self.counter.world[c['id']] for c in ordered],
                result['required_sides'],pose,pose['yaw_rad']+arguments['lane_heading_rad'],t)
            self._set_reference(result['path'],pose,arguments)
        else:
            if self._progress()['complete'] and not self.reference_returns_to_lane:
                # If the initial visible region could only support holding the
                # exit side, add one finite recovery path. Never chase a moving
                # distant lane-centre target on every frame.
                coverage=arguments['coverage_forward_m']
                result={'action':'stop','path':[],'reason':'no_lane_recovery_route'}
                if numeric(coverage):
                    for distance in np.arange(1.,coverage+.25,.25):
                        candidate=self.planner.plan([],**{**arguments,'coverage_forward_m':min(distance,coverage)},required_sides={})
                        if candidate['action']=='stop':continue
                        self._set_reference(candidate['path'],pose,arguments)
                        if self.reference_returns_to_lane:break
                if not self.reference_returns_to_lane:
                    self.phase='waiting_for_clear_route';return {**stop,**result,'phase':self.phase}
            result=self._follow_reference(pose,cones,arguments)
            if result['reason']=='slalom_reference_tracking_error':
                self.phase='fault';return {**stop,**result,'phase':self.phase}
        if result['action']=='stop':self.phase='waiting_for_clear_route'
        elif self.phase!='return_to_lane':self.phase='pass_first_cone' if not self.slalom.passed else 'pass_second_cone'
        return {**result,'phase':self.phase,'passed_ids':self._progress()['passed_ids'],
                'slalom':self._progress(),'requires_verified_execution_adapter':True}
