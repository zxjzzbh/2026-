"""Configured red/blue cone candidates and tracking, without actuator imports.

A large/clipped selected-color shape remains an unresolved obstruction rather
than silently disappearing or being counted as a passed cone.
"""
import math

import cv2
import numpy as np


def iou(a,b):
    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area
    return area/union if union>0 else 0.


def obstacle_color(settings):
    scene=settings.get('scene',{})
    color=scene.get('obstacle_color','red' if scene.get('surface')=='blue_running_track' else 'blue')
    if color not in ('red','blue'):raise ValueError('obstacle_color must be red or blue')
    return color


class ConeDetector:
    def __init__(self, settings):
        self.color=obstacle_color(settings)
        self.lower=np.asarray(settings['blue_hsv_lower'],np.uint8)
        self.upper=np.asarray(settings['blue_hsv_upper'],np.uint8)

    def detect(self,image):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        if self.color=='red':
            mask=cv2.bitwise_or(cv2.inRange(hsv,(0,110,50),(10,255,255)),
                               cv2.inRange(hsv,(170,110,50),(179,255,255)))
        else:
            mask=cv2.inRange(hsv,self.lower,self.upper)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        h,w=mask.shape
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        rows=[]
        for contour in contours:
            x,y,bw,bh=cv2.boundingRect(contour);area=float(cv2.contourArea(contour))
            if bw<max(8,w*.02) or bh<max(12,h*.035) or area<max(25,w*h*.00015) or y+bh<h*.25:continue
            clipped=x<=1 or y<=1 or x+bw>=w-1 or y+bh>=h-1
            local=np.zeros((bh,bw),np.uint8)
            cv2.drawContours(local,[contour-np.array([[[x,y]]],np.int32)],-1,1,-1)
            widths=np.count_nonzero(local,axis=1)
            broad=np.flatnonzero(widths>=widths.max()*.9)
            base=int(np.median(broad))
            start=max(0,round((base+1)*.1));end=max(start+1,round((base+1)*.3))
            upper=float(np.mean(widths[start:end]))
            lower=float(np.mean(widths[round((base+1)*.8):base+1]))
            hull=float(cv2.contourArea(cv2.convexHull(contour)))
            tapered=(base>=bh*.55 and lower>=max(1,upper)*1.5 and .7<=bh/bw<=4
                     and .28<=area/(bw*bh)<=.85 and hull>0 and area/hull>=.75)
            perimeter=cv2.arcLength(contour,True)
            rounded=(.75<=bh/bw<=1.35 and .67<=area/(bw*bh)<=.9 and hull>0 and area/hull>=.9
                     and perimeter>0 and 4*math.pi*area/(perimeter*perimeter)>=.72)
            kind='cone_candidate' if (tapered or rounded) and not clipped else 'unresolved_'+self.color
            # Small non-cone colored patches are not used as road obstacles;
            # close/large/edge-intersecting patches remain visible to the user.
            if kind!='cone_candidate' and not (clipped or area/(w*h)>=.012 or y+bh>=h*.8):continue
            bottom=np.flatnonzero(local[max(0,base-1):min(bh,base+2)].any(axis=0))
            foot_x=x+float(np.mean(bottom)) if len(bottom) else x+bw/2
            rows.append({'kind':kind,'obstacle_color':self.color,'bbox_xyxy':[x,y,x+bw,y+bh],
                         'foot_pixel':[foot_x,float(y+base)],'clipped':clipped,
                         'shape_evidence':'tapered' if tapered else 'round_top_view' if rounded else 'unresolved',
                         'ground_contact_usable':kind=='cone_candidate' and tapered and not clipped,
                         'area_fraction':area/(w*h),'position_in_image':
                            'left' if foot_x<w*.4 else 'right' if foot_x>w*.6 else 'center'})
        rows.sort(key=lambda r:r['bbox_xyxy'][3],reverse=True)
        return rows,mask


# Existing callers keep the same import; color now follows explicit scene
# configuration rather than being inferred from the legacy class name.
BlueConeDetector = ConeDetector


class ImageConeTracker:
    """Per-view IDs only. Two camera IDs never imply two physical cones."""
    def __init__(self,camera,settings,*,near_field_recovery=False):
        self.camera,self.s=camera,settings
        self.tracks={};self.next_id=1;self.last_id=self.last_t=None
        self.near_field_recovery=False
        self._history={};self._pending={};self._exhausted=set()
        self.latest_diagnostics={'enabled':False}
        self.set_near_field_recovery(near_field_recovery)

    def set_near_field_recovery(self,enabled):
        if type(enabled) is not bool:raise ValueError('near_field_recovery must be a boolean')
        if self.near_field_recovery!=enabled:
            self._history.clear();self._pending.clear();self._exhausted.clear()
            if not enabled:
                for row in self.tracks.values():
                    for key in ('identity_revision','identity_recovered','identity_recovery'):row.pop(key,None)
        self.near_field_recovery=enabled
        self.latest_diagnostics={'enabled':enabled}

    def _color(self,row):
        color=row.get('obstacle_color')
        if color is None and row.get('kind','').startswith('unresolved_'):color=row['kind'].split('_',1)[1]
        return color or obstacle_color(self.s)

    def _normal_matches(self,detections,t):
        if not self.near_field_recovery:
            # Default callers retain their original ordered IoU policy.
            used=set();normal={}
            for i,detection in enumerate(detections):
                matches=[(iou(track['bbox_xyxy'],detection['bbox_xyxy']),key)
                    for key,track in self.tracks.items()
                    if key not in used and t-track['seen_s']<=self.s['track_timeout_s']]
                score,key=max(matches,default=(0,None))
                if score>=.15:normal[i]=key;used.add(key)
            return normal,None
        keys=sorted(key for key,track in self.tracks.items()
            if t-track['seen_s']<=self.s['track_timeout_s'])
        scores=np.asarray([[iou(row['bbox_xyxy'],self.tracks[key]['bbox_xyxy'])
            if self._color(row)==self._color(self.tracks[key]) else 0.
            for key in keys] for row in detections],float)
        rows=list(range(len(detections)));columns=list(range(len(keys)));normal={}
        # Resolve all clearly mutual-best pairs together before considering the
        # remainder. A close competitor is left unmatched, never an ID tiebreak.
        while rows and columns:
            matrix=scores[np.ix_(rows,columns)];strong=[]
            for r,index in enumerate(rows):
                c=int(np.argmax(matrix[r]));score=float(matrix[r,c])
                if score<.15 or int(np.argmax(matrix[:,c]))!=r:continue
                runner_row=float(np.sort(matrix[r])[-2]) if len(columns)>1 else 0.
                runner_column=float(np.sort(matrix[:,c])[-2]) if len(rows)>1 else 0.
                if score-runner_row>=.05-1e-12 and score-runner_column>=.05-1e-12:
                    strong.append((index,columns[c]))
            if not strong:break
            for index,column in strong:normal[index]=keys[column]
            row_used={r for r,_ in strong};column_used={c for _,c in strong}
            rows=[r for r in rows if r not in row_used]
            columns=[c for c in columns if c not in column_used]
        ambiguous=sum(any(scores[r,c]>=.15 for c in columns) for r in rows) if columns else 0
        return normal,{'method':'mutual_best_iou','minimum_iou':.15,'minimum_margin':.05,
            'matched':len(normal),'unmatched_detections':len(detections)-len(normal),
            'ambiguous_detections':ambiguous}

    def _sample(self,row,frame_id,t):
        try:
            box=np.asarray(row['bbox_xyxy'],float);foot=np.asarray(row['foot_pixel'],float)
        except (KeyError,TypeError,ValueError):return None
        w,h=self.s.get('image_size',[480,360])
        if (box.shape!=(4,) or foot.shape!=(2,) or not np.isfinite(box).all()
                or not np.isfinite(foot).all() or not 0<=box[0]<box[2]<=w
                or not 0<=box[1]<box[3]<=h or not 0<=foot[0]<w or not 0<=foot[1]<h):return None
        return {'box':box,'foot':foot,'frame_id':frame_id,'t':t,
            'color':self._color(row)}

    def _moving_near(self,key):
        history=self._history.get(key,[])
        if len(history)<3:return False
        _,h=self.s.get('image_size',[480,360]);first,last=history[0],history[-1]
        near=.85 if self.camera=='primary' else .70
        ys=[p['foot'][1] for p in history]
        return bool(last['foot'][1]>=h*near and last['t']-first['t']>=.10
            and last['foot'][1]-first['foot'][1]>=h*.015
            and max(np.diff([p['t'] for p in history]))<=self.s['max_frame_age_s']
            and min(np.diff(ys))>=-h*.02)

    def _compatible(self,key,current,pending):
        """Local current-observation association, not a predicted visible row."""
        history=self._history[key]
        previous=pending.get('sample') or history[-1]
        before=history[-1] if pending.get('sample') else history[-2]
        dt=previous['t']-before['t'];gap=current['t']-previous['t']
        if dt<=0 or not 0<gap<=self.s['max_frame_age_s']:return False
        if current['color']!=previous['color']:return False
        w,h=self.s.get('image_size',[480,360])
        old_size=previous['box'][2:]-previous['box'][:2]
        new_size=current['box'][2:]-current['box'][:2]
        ratios=new_size/old_size;area=float(np.prod(ratios))
        if np.any(ratios<.5) or np.any(ratios>2.8) or not .3<=area<=5.:
            return False
        if float(np.prod(new_size))>w*h*.15:return False
        velocity=(previous['foot']-before['foot'])/dt
        predicted=previous['foot']+velocity*gap
        tolerance=np.maximum([12.,12.],np.maximum(old_size,new_size)*[.9,.7])
        if np.any(np.abs(current['foot']-predicted)>tolerance):return False
        dy=current['foot'][1]-previous['foot'][1]
        if not -h*.025<=dy<=h*.18:return False
        # Even an enlarged candidate must remain in the old cone's local area.
        old=history[-1]
        if abs(current['foot'][0]-old['foot'][0])>max(24.,float(new_size[0])*1.5):return False
        return True

    def _recover(self,detections,normal,frame_id,t,previous_frame):
        diagnostics={'enabled':True,'source_camera':self.camera,'frame_id':frame_id,
            'captured_s':t,'pending':[],'pending_omitted':0,
            'recovered':[],'recovered_omitted':0,'rejected':{}}
        self.latest_diagnostics=diagnostics
        def reject(reason):
            counts=diagnostics['rejected'];counts[reason]=counts.get(reason,0)+1
        matched=set(normal.values())
        for key in matched:
            self._pending.pop(key,None);self._exhausted.discard(key)
        eligible=[key for key,row in self.tracks.items() if key not in matched
            and row.get('confirmed') and key not in self._exhausted
            and t-row['seen_s']<=self.s['track_timeout_s'] and self._moving_near(key)]
        eligible.sort(key=lambda key:self.tracks[key]['seen_s'],reverse=True)
        samples={i:self._sample(row,frame_id,t) for i,row in enumerate(detections)}
        candidates={};live=[]
        # Every possible owner participates in the exclusion check. The cap
        # below limits diagnostics, not evidence of a competing old identity.
        for key in eligible:
            pending=self._pending.setdefault(key,{'first_s':t,'frames':[],
                'sample':None,'provisional_ids':set(),'current_index':None})
            pending['current_index']=None
            if t-pending['first_s']>.25+1e-9:
                self._exhausted.add(key);self._pending.pop(key,None);reject('budget_expired');continue
            if pending.get('last_frame')!=previous_frame:
                pending['frames']=[];pending['sample']=None;pending['provisional_ids'].clear()
            options=[]
            for i,row in enumerate(detections):
                assigned=normal.get(i)
                if assigned and self.tracks[assigned].get('confirmed'):continue
                sample=samples[i]
                if (sample is not None and row.get('kind') in ('cone_candidate','unresolved_'+sample['color'])
                        and self._compatible(key,sample,pending)):options.append(i)
            candidates[key]=options;live.append(key)
        owners={i:[key for key in live if i in candidates[key]]
            for i in range(len(detections))}
        recovered={};absorbed=set()
        for key in live:
            pending=self._pending[key];options=candidates[key]
            over_capacity=len(options)>8 or any(len(owners[i])>8 for i in options)
            if over_capacity or len(options)!=1 or len(owners[options[0]])!=1:
                pending['frames']=[];pending['sample']=None;pending['provisional_ids'].clear()
                pending['last_frame']=frame_id
                reject('association_capacity' if over_capacity else 'ambiguous' if options else 'no_matching_current_detection')
            else:
                if not pending['frames']:pending['support_first_s']=t
                i=options[0];pending['frames'].append(frame_id);pending['sample']=samples[i]
                pending['last_frame']=frame_id;pending['current_index']=i
                if len(pending['frames'])>=2:
                    ids={identity for identity in pending['provisional_ids']
                        if identity not in normal.values() or normal.get(i)==identity}
                    if normal.get(i) and normal[i]!=key:ids.add(normal[i])
                    event={'source_camera':self.camera,'anchor_frame_id':self._history[key][-1]['frame_id'],
                        'evidence_frame_ids':pending['frames'][-2:],
                        'budget_started_s':pending['first_s'],
                        'first_candidate_s':pending['support_first_s'],'recovered_s':t,
                        'absorbed_ids':sorted(ids),'ground_evidence':False}
                    recovered[i]=(key,event);absorbed.update(ids)
                    if len(diagnostics['recovered'])<8:diagnostics['recovered'].append({'id':key,**event})
                    else:diagnostics['recovered_omitted']+=1
                    self._history[key]=[];self._pending.pop(key,None)
            if key in self._pending:
                if len(diagnostics['pending'])<8:
                    diagnostics['pending'].append({'id':key,'first_s':pending['first_s'],
                        'age_s':t-pending['first_s'],'support_frames':len(pending['frames'])})
                else:diagnostics['pending_omitted']+=1
        return recovered,absorbed

    def update(self,detections,frame_id,captured_s):
        if (type(frame_id) is not int or not isinstance(captured_s,(int,float)) or not math.isfinite(captured_s)
                or self.last_id is not None and frame_id<=self.last_id
                or self.last_t is not None and captured_s<=self.last_t):
            raise ValueError('tracking requires distinct ordered camera frames')
        if self.last_t is not None and captured_s-self.last_t>self.s['max_frame_age_s']:
            self.tracks.clear()
            self._history.clear();self._pending.clear();self._exhausted.clear()
        previous_frame=self.last_id
        self.last_id,self.last_t=frame_id,captured_s
        normal,normal_diagnostics=self._normal_matches(detections,captured_s)
        recovered,absorbed=({},set())
        if self.near_field_recovery:
            recovered,absorbed=self._recover(detections,normal,frame_id,captured_s,previous_frame)
            self.latest_diagnostics['normal_matching']=normal_diagnostics
        used=set();output=[];assigned={}
        for i,detection in enumerate(detections):
            recovery=recovered.get(i);key=recovery[0] if recovery else normal.get(i)
            if key is None:
                key=f'{self.camera}-{self.next_id}';self.next_id+=1
                track={'id':key,'first_s':captured_s,'hits':0,'confirmed':False}
            else:track=dict(self.tracks[key])
            used.add(key);assigned[i]=key
            if detection['kind']=='cone_candidate':track['hits']+=1
            track.update(detection,seen_s=captured_s,visible=True)
            track['confirmed']=(track['confirmed'] or detection['kind']=='cone_candidate'
                and track['hits']>=self.s['confirm_frames'] and captured_s-track['first_s']>=self.s['confirm_s'])
            if self.near_field_recovery:
                track['identity_revision']=track.get('identity_revision',0)+(1 if recovery else 0)
                track['identity_recovered']=bool(recovery)
                track.pop('identity_recovery',None)
                if recovery:
                    track['identity_recovery']=recovery[1]
                    track['ground_contact_usable']=False
                sample=self._sample(track,frame_id,captured_s)
                if track['confirmed'] and sample is not None:
                    history=self._history.setdefault(key,[]);history.append(sample)
                    self._history[key]=history[-3:]
            # A clipped continuation keeps its identity but has no new metric
            # contact point. It is never interpreted as already passed.
            self.tracks[key]=track;output.append(dict(track))
        for pending in self._pending.values():
            if pending.get('current_index') in assigned:
                pending['provisional_ids'].add(assigned[pending['current_index']])
        for key in absorbed:
            if key not in used:self.tracks.pop(key,None)
        for key,track in list(self.tracks.items()):
            if key in used:continue
            if captured_s-track['seen_s']>self.s['track_timeout_s']:
                del self.tracks[key]
            else:
                invisible={**track,'visible':False,'ground_contact_usable':False}
                if self.near_field_recovery:
                    invisible['identity_recovered']=False;invisible.pop('identity_recovery',None)
                output.append(invisible)
        for states in (self._history,self._pending):
            for key in list(states):
                if key not in self.tracks:states.pop(key,None)
        self._exhausted.intersection_update(self.tracks)
        return output
