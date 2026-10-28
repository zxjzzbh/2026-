"""Real secondary-camera observations -> measured pose, corridor and cone route.

No motor I/O here. The local world origin is the vehicle rear axle when reset;
it need not be a surveyed GPS position. Optical-flow validity is never inferred
from throttle, and an invalid pose cannot quietly reset while driving.
"""

from .steering import LEFT_US, RIGHT_US, RADIUS_ALIASES, radius_settings
import copy
import math

import cv2
import numpy as np

from .autonomy_profile import GroundProjection,fit_ground_calibration
from .brake_parking import numeric
from .cone_planning import ConeMission,readiness,distance_to_path


DEFAULT_DRIVING={
    'forward_us':1575,'max_run_s':30.,'max_speed_mps':.2,'lookahead_m':.4,
    'brake_pulse_s':.12,'brake_release_s':.1,'actuator_lease_s':.2,
    'steering_radius_right_cm':None,'steering_radius_left_cm':None,
    'visual_error_bound_m':None,'visual_verified_distance_m':None,
    'fixed_camera_confirmed':False,'steering_verified':False,'odometry_verified':False,
}


def driving_settings(settings):
    return {**DEFAULT_DRIVING,**radius_settings(settings.get('driving',{}))}


def validate_driving(settings):
    d=driving_settings(settings)
    for key,lo,hi in [('forward_us',1501,1625),('max_run_s',1,60),('max_speed_mps',.02,.5),('lookahead_m',.2,1.)]:
        if not numeric(d[key]) or not lo<=d[key]<=hi:raise ValueError('避障参数越界：'+key)
    if type(d['forward_us']) is not int:raise ValueError('油门应为整数微秒')
    for key in ('fixed_camera_confirmed','steering_verified','odometry_verified'):
        if type(d[key]) is not bool:raise ValueError(key+' must be boolean')
    for key in ('steering_radius_right_cm','steering_radius_left_cm'):
        if d[key] is not None and (not numeric(d[key]) or not 20<=abs(d[key])<=5000):
            raise ValueError('转弯半径用 cm：左负右正，绝对值 20–5000')
    if d['steering_radius_right_cm'] is not None and d['steering_radius_right_cm']<=0:
        raise ValueError('1350 右转半径应为正值，不能沿用旧档位标定')
    if d['steering_radius_left_cm'] is not None and d['steering_radius_left_cm']>=0:
        raise ValueError('1870 左转半径应为负值，不能沿用旧档位标定')
    if all(d[k] is not None for k in ('steering_radius_right_cm','steering_radius_left_cm')):
        if d['steering_radius_right_cm']*d['steering_radius_left_cm']>=0:raise ValueError('两侧转向半径应一左一右')
    for key,lo,hi in [('visual_error_bound_m',.001,.1),('visual_verified_distance_m',.2,20)]:
        if d[key] is not None and (not numeric(d[key]) or not lo<=d[key]<=hi):raise ValueError(key)
    # Keep the already-used finite F/B pulse and watchdog envelope.
    if (d['brake_pulse_s']!=.12 or d['brake_release_s']!=.1 or d['actuator_lease_s']!=.2):
        raise ValueError('避障沿用已审查的有限 F/B 刹车及租约参数')
    return d


def execution_readiness(settings):
    report=readiness(settings);missing=list(report['missing']);d=validate_driving(settings)
    for key in ('fixed_camera_confirmed','steering_verified','odometry_verified'):
        if d[key] is not True:missing.append('driving.'+key)
    for key in ('steering_radius_right_cm','steering_radius_left_cm','visual_error_bound_m','visual_verified_distance_m'):
        if d[key] is None:
            missing.append('driving.'+key)
            if key in RADIUS_ALIASES:missing.append('driving.'+RADIUS_ALIASES[key])
    if d['visual_error_bound_m'] is not None and numeric(settings['motion'].get('maximum_tracking_error_m')):
        if d['visual_error_bound_m']>settings['motion']['maximum_tracking_error_m']:missing.append('driving.visual_error_exceeds_budget')
    return {'ready':not missing,'metric_planning_ready':report['metric_planning_ready'],'missing':missing,'field_verified':False}


def actuator_settings(settings):
    d=validate_driving(settings)
    result={'pwm':{'neutral_us':1500,'steering_center_us':1610,'search_us':d['forward_us'],
                   'creep_us':d['forward_us'],'brake_us':1400},
            'brake_pulse_s':d['brake_pulse_s'],'brake_release_s':d['brake_release_s'],'actuator_lease_s':d['actuator_lease_s']}
    if settings.get('low_speed_test'):
        result['cone_forward_us']=sorted({min(d['forward_us'],1570),min(d['forward_us'],1580)})
    return result


def import_ground_calibration(settings,data):
    if not isinstance(data,dict) or data.get('measured_fixed_camera') is not True:
        raise ValueError('地面标定需确认坐标实测且第二路摄像头固定')
    if data.get('camera')!='secondary' or data.get('image_size')!=[480,360]:
        raise ValueError('仅接受第二路 H65 480×360 原始预览坐标')
    try:result=fit_ground_calibration(data)
    except (KeyError,TypeError,ValueError,cv2.error) as exc:raise ValueError('地面标定格式或点位无效：'+str(exc)) from exc
    bound=data.get('accepted_error_m')
    if not numeric(bound) or not .001<=bound<=.03 or result['independent_check_max_error_m']>bound:
        raise ValueError('独立检查点误差超过填写值；误差上限须为 0.1–3 cm')
    result.update(verified=True,coordinate_frame='rear_axle_x_right_y_forward',max_error_m=bound,
                  calibration_evidence=copy.deepcopy(data))
    candidate=copy.deepcopy(settings);candidate['ground']=result
    candidate.setdefault('driving',{})['fixed_camera_confirmed']=True
    return candidate


def reuse_legacy_ground(settings,data):
    """Load the accepted historical mapping without inventing a current check.

    The repository reference already includes the 640x480 -> 480x360 pixel
    transform and the measured rear-axle origin correction. Its original
    polygon must remain intact; loading cannot authorize extrapolation/motion.
    """
    if (not isinstance(data,dict) or data.get('camera')!='secondary'
            or data.get('image_size')!=[480,360]
            or data.get('source_commit')!='6f93ea8858bfe83676bc4bdbfd790de7979de7ec'
            or data.get('candidate_only') is not True):
        raise ValueError('旧标定来源或第二路图像规格不一致')
    if settings['ground'].get('verified') is True:
        raise ValueError('已有当前确认的地面标定，不用旧参考覆盖')
    bound=data.get('accepted_error_m')
    result=fit_ground_calibration(data)
    if (not numeric(bound) or not .001<=bound<=.03
            or result['independent_check_max_error_m']>bound
            or not numeric(data.get('source_repeat_error_m'))
            or data['source_repeat_error_m']>bound):
        raise ValueError('旧标定复算误差不满足其原验收范围')
    result.update(coordinate_frame='rear_axle_x_right_y_forward',max_error_m=bound,
        historical_verified=True,current_pose_verified=False,
        reuse_source={'commit':data['source_commit'],'path':data['source_path'],
                     'pose_us':data['source_pose_us'],'source_repeat_error_m':data['source_repeat_error_m']},
        calibration_evidence=copy.deepcopy(data))
    candidate=copy.deepcopy(settings);candidate['ground']=result
    candidate.setdefault('driving',{})['fixed_camera_confirmed']=False
    return candidate


class ConeOdometry:
    def __init__(self,settings):
        self.s=settings;self.d=driving_settings(settings);self.projection=GroundProjection(settings['ground'])
        self.previous=None;self.pose={'x_m':0.,'y_m':0.,'yaw_rad':0.}
        self.failed=False;self.travel=0.

    def update(self,image,frame_id,t,tracks,context):
        bad={'valid':False,'pose':None,'speed_mps':None,'reason':'optical_feedback_unavailable'}
        if self.failed:return {**bad,'reason':'pose_lost_reprepare_required'}
        grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);mask=np.zeros(grey.shape,np.uint8)
        cv2.fillConvexPoly(mask,self.projection.poly.astype(np.int32),255)
        if context['valid']:
            corridor=np.zeros_like(mask)
            cv2.fillPoly(corridor,[np.asarray(context['left_inner']+context['right_inner'][::-1],np.int32)],255)
            mask=cv2.bitwise_and(mask,corridor)
        else:
            self.failed=True;return {**bad,'reason':'white_inner_edges_unavailable'}
        for track in tracks:
            x1,y1,x2,y2=track['bbox_xyxy'];cv2.rectangle(mask,(max(0,x1-5),max(0,y1-5)),(x2+5,y2+5),0,-1)
        points=cv2.goodFeaturesToTrack(grey,maxCorners=160,qualityLevel=.01,minDistance=6,mask=mask)
        old=self.previous;self.previous=(grey,points,t,frame_id)
        if old is None:return {**bad,'reason':'collecting_ground_motion'}
        old_grey,old_points,old_t,old_id=old;dt=t-old_t
        if old_points is None or points is None or not 0<dt<=.25 or frame_id<=old_id:
            self.failed=True;return bad
        moved,status,errors=cv2.calcOpticalFlowPyrLK(old_grey,grey,old_points,None,winSize=(21,21),maxLevel=2)
        if moved is None or status is None or errors is None:self.failed=True;return bad
        before=[];after=[]
        for p,q,good,error in zip(old_points[:,0],moved[:,0],status[:,0],errors[:,0]):
            if not good or error>20:continue
            x,y=np.rint(q).astype(int)
            if not (0<=y<mask.shape[0] and 0<=x<mask.shape[1] and mask[y,x]):continue
            try:
                a=self.projection.point(p);b=self.projection.point(q)
            except ValueError:continue
            # Append both only after both projections succeeded.
            before.append(a);after.append(b)
        if len(before)<20:self.failed=True;return bad
        before=np.asarray(before,np.float64);after=np.asarray(after,np.float64)
        affine,inliers=cv2.estimateAffinePartial2D(before,after,method=cv2.RANSAC,ransacReprojThreshold=.015,maxIters=250)
        if affine is None or inliers is None or np.mean(inliers)<.75:self.failed=True;return bad
        scale=math.hypot(affine[0,0],affine[1,0])
        if not .98<=scale<=1.02:self.failed=True;return bad
        rotation=affine[:,:2]/scale;delta=-rotation.T@affine[:,2];yaw=math.atan2(rotation[1,0],rotation[0,0])
        fitted=before@affine[:,:2].T+affine[:,2];keep=inliers[:,0]>0
        rms=float(np.sqrt(np.mean(np.sum((fitted[keep]-after[keep])**2,axis=1))))
        speed=float(delta[1]/dt);self.travel+=float(np.linalg.norm(delta))
        if (rms>.015 or np.linalg.norm(delta)/dt>self.d['max_speed_mps']*1.5
                or abs(yaw/dt)>1.5 or self.travel>self.d['visual_verified_distance_m']):
            self.failed=True;return {**bad,'reason':'motion_limit_or_calibrated_distance_exceeded'}
        heading=self.pose['yaw_rad'];co,si=math.cos(heading),math.sin(heading)
        self.pose['x_m']+=float(co*delta[0]+si*delta[1]);self.pose['y_m']+=float(-si*delta[0]+co*delta[1]);self.pose['yaw_rad']+=yaw
        pose={**self.pose,'valid':True,'error_bound_m':self.d['visual_error_bound_m']}
        return {'valid':True,'pose':pose,'speed_mps':speed,'travel_m':self.travel,'rms_m':rms,'reason':'measured_ground_flow'}


def metric_corridor(context,settings):
    if not context.get('valid'):raise ValueError('未确认所选场地及左右白线内侧')
    projection=GroundProjection(settings['ground']);sides=[]
    for key in ('left_inner','right_inner'):
        points=[]
        for pixel in context[key]:
            try:points.append(projection.point(pixel))
            except ValueError:continue
        if len(points)<6:raise ValueError('白线不在有效地面标定区域内')
        a=np.asarray(points);fit=np.polyfit(a[:,1],a[:,0],1)
        if np.ptp(a[:,1])<.3 or np.max(np.abs(np.polyval(fit,a[:,1])-a[:,0]))>settings['ground']['max_error_m']+.015:
            raise ValueError('白线米制拟合不稳定')
        sides.append((fit,float(a[:,1].max())))
    slope=float((sides[0][0][0]+sides[1][0][0])/2);heading=math.atan(slope)
    if abs(heading)>.4 or abs(sides[0][0][0]-sides[1][0][0])>.08:raise ValueError('直道方向或平行白线不一致')
    co=math.cos(heading);bounds=[float(sides[i][0][1]*co) for i in range(2)]
    width=bounds[1]-bounds[0]
    if abs(width-settings['geometry']['lane_width_m'])>max(.05,2*settings['ground']['max_error_m']):raise ValueError('白线宽度与实测赛道宽度不符')
    return bounds,heading,min(x[1] for x in sides)


class ConeNavigator:
    def __init__(self,settings):
        self.s=copy.deepcopy(settings);self.report=execution_readiness(settings)
        d=driving_settings(settings)
        try:
            self.odometry=ConeOdometry(settings) if d['visual_error_bound_m'] is not None and d['visual_verified_distance_m'] is not None else None
        except (ValueError,TypeError):self.odometry=None
        self.mission=ConeMission(settings);self.last_id=None;self.latest=None
        self.previous_route=None

    def update(self,image,row,now):
        if self.odometry is None:
            self.latest={'action':'stop','phase':'needs_calibration','missing':self.report['missing'],'reason':'避障标定尚未补齐','hardware_output':False}
            return self.latest
        if row['frame_id']==self.last_id:return self.latest
        self.last_id=row['frame_id']
        if not 0<=now-row['captured_s']<=self.s['max_frame_age_s']:raise ValueError('避障图像已过期')
        bounds,heading,coverage=metric_corridor(row['road_context'],self.s)
        motion=self.odometry.update(image,row['frame_id'],row['captured_s'],row['tracks'],row['road_context'])
        if not motion['valid']:
            self.latest={'action':'stop','phase':'waiting_for_feedback','reason':motion['reason'],'motion':motion,'hardware_output':False}
            return self.latest
        pose=motion['pose'];co,si=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad']);world=[];unresolved=[]
        if not self.report['ready']:
            self.latest={'action':'stop','phase':'calibration_only','missing':self.report['missing'],
                'reason':'地面位移标定观察：仅计算，不驱动车辆','motion':motion,'captured_s':row['captured_s'],'hardware_output':False}
            return self.latest
        if self.previous_route is not None:
            prior_pose,path=self.previous_route
            dx=pose['x_m']-prior_pose['x_m'];dy=pose['y_m']-prior_pose['y_m']
            co0,si0=math.cos(prior_pose['yaw_rad']),math.sin(prior_pose['yaw_rad'])
            px,py=co0*dx-si0*dy,si0*dx+co0*dy
            error,_=distance_to_path([{'x_m':0.,'y_m':0.}]+path,px,py)
            if error>self.s['motion']['maximum_tracking_error_m']:
                raise ValueError('实测车辆位置偏离上一条路径，停止并重新准备')
        projection=self.odometry.projection
        for track in row['tracks']:
            if track['visible'] and track['confirmed'] and track['ground_contact_usable']:
                try:x,y=projection.point(track['foot_pixel'])
                except ValueError:unresolved.append(track['id']);continue
                world.append({'id':track['id'],'x_m':pose['x_m']+co*x+si*y,'y_m':pose['y_m']-si*x+co*y,
                              'radius_m':self.s['geometry']['cone_base_width_m']/2})
            elif track['id'] not in self.mission.counter.aliases:unresolved.append(track['id'])
        sample={'frame_id':row['frame_id'],'captured_s':row['captured_s'],'pose':pose,'observed_world':world,
                'unresolved_obstacles':unresolved,'lane_bounds_m':bounds,'lane_heading_rad':heading,'lane_valid':True,
                'lane_offset_m':-sum(bounds)/2,'coverage_forward_m':coverage}
        result=self.mission.step(sample,now)
        if result.get('path'):self.previous_route=(dict(pose),result['path'])
        self.latest={**result,'motion':motion,'captured_s':row['captured_s'],'sample':sample}
        return self.latest

    def draw(self,image):
        output=image.copy();result=self.latest or {}
        if not result.get('path'):return output
        inverse=np.linalg.inv(self.s['ground']['homography']);pixels=[]
        for point in result['path']:
            p=inverse@np.array([point['x_m'],point['y_m'],1.])
            if abs(p[2])<1e-9:continue
            x,y=p[:2]/p[2]
            if np.isfinite(x+y) and 0<=x<image.shape[1] and 0<=y<image.shape[0]:pixels.append([round(x),round(y)])
        if len(pixels)>1:cv2.polylines(output,[np.asarray(pixels,np.int32)],False,(255,0,255),2)
        return output


def steering_for_path(path,settings):
    d=driving_settings(settings)
    if d['steering_radius_right_cm'] is None or d['steering_radius_left_cm'] is None:
        raise ValueError('1350／1870 新档位尚无有效曲率映射，旧档位半径不能代用')
    choices=[p for p in path if p['y_m']>0]
    if not choices:raise ValueError('没有前方可跟随路径')
    p=min(choices,key=lambda p:abs(math.hypot(p['x_m'],p['y_m'])-d['lookahead_m']))
    curvature=2*p['x_m']/(p['x_m']**2+p['y_m']**2)
    mapping=sorted([(100/d['steering_radius_right_cm'],RIGHT_US),(0.,1610),(100/d['steering_radius_left_cm'],LEFT_US)])
    if not mapping[0][0]<=curvature<=mapping[-1][0]:raise ValueError('所需转向超过实测转向范围')
    return int(round(np.interp(curvature,[p[0] for p in mapping],[p[1] for p in mapping])))


class ConeDrivingController:
    def __init__(self,settings,now):
        self.s=settings;self.d=driving_settings(settings);self.started=now
        self.phase='arming';self.stop_started=self.still_since=None
        self.brake_count=0;self.brake_phase=None;self.brake_deadline=None
        self.pause_started=self.pause_brake_until=self.pause_release_until=None

    def step(self,navigation,now):
        intent={'action':'neutral','esc_us':1500,'steering_us':1610,'phase':self.phase,'reason':'3 秒中位，随后开始避障'}
        if now-self.started>self.d['max_run_s']+3:raise ValueError('避障总时限已到')
        motion=navigation.get('motion') or {}
        if not motion.get('valid') or not 0<=now-navigation.get('captured_s',-1e9)<=.25:raise ValueError('避障位移或图像反馈失效')
        speed=motion['speed_mps']
        if abs(speed)>self.d['max_speed_mps']:raise ValueError('实测速度超过避障速度上限')
        if speed<-.025:raise ValueError('观测到反向移动，结束测试')
        if now-self.started<3:
            if abs(speed)>.025:raise ValueError('中位初始化时车辆仍在移动')
            return intent
        if navigation.get('phase')=='complete' or self.stop_started is not None:
            if self.stop_started is None:
                self.stop_started=now;self.brake_count=1;self.brake_phase='brake';self.brake_deadline=now+self.d['brake_pulse_s']
            elapsed=now-self.stop_started
            if abs(speed)<=.025:
                if self.still_since is None:self.still_since=now
            else:self.still_since=None
            if self.still_since is not None and now-self.still_since>=.4:
                self.phase='complete';return {**intent,'phase':'complete','reason':'已通过两桶并回归赛道，保持停车'}
            if elapsed>3:raise ValueError('制动后仍未得到停稳反馈')
            if now>=self.brake_deadline:
                if self.brake_phase=='brake':
                    self.brake_phase='release';self.brake_deadline=now+self.d['brake_release_s']
                elif self.brake_count<3:
                    self.brake_count+=1;self.brake_phase='brake';self.brake_deadline=now+self.d['brake_pulse_s']
            brake=self.brake_phase=='brake'
            self.phase='braking'
            return {**intent,'phase':'braking','reason':'两桶结束，主动制动并核对停稳',
                    'action':'brake' if brake else 'neutral','esc_us':1400 if brake else 1500}
        if navigation.get('action')!='follow_candidate_path' or self.pause_started is not None:
            if navigation.get('action')!='follow_candidate_path' and navigation.get('phase') not in ('waiting_for_feedback','waiting_for_clear_route'):
                raise ValueError('当前没有可通过路线：'+str(navigation.get('reason')))
            if self.pause_started is None:
                self.pause_started=now;self.pause_brake_until=now+self.d['brake_pulse_s'];self.pause_release_until=None
            if now-self.pause_started>3:raise ValueError('障碍或可通过路线持续未确认，结束本轮')
            if now<self.pause_brake_until:
                return {**intent,'action':'brake','esc_us':1400,'phase':'waiting_for_clear_path','reason':'暂未确认障碍／路线，先制动等待新帧'}
            if self.pause_release_until is None:self.pause_release_until=now+self.d['brake_release_s']
            if navigation.get('action')!='follow_candidate_path' or now<self.pause_release_until or abs(speed)>.025:
                return {**intent,'phase':'waiting_for_clear_path','reason':'保持中位，等待明确路线与停稳反馈'}
            self.pause_started=self.pause_brake_until=self.pause_release_until=None
        self.phase=navigation['phase']
        return {**intent,'phase':self.phase,'reason':'根据第二路实景路径低速避障',
                'action':'search','esc_us':self.d['forward_us'],'steering_us':steering_for_path(navigation['path'],self.s)}
