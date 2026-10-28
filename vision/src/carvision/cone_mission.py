"""Bounded task evidence; changing the steering target cannot clear a task.

Only the main camera registers/counts task cones. An observation timeout is
never a passed/rejected resolution. Overflow blocks completion, not movement.
"""
from .brake_output import numeric


class ConeMissionLedger:
    def __init__(self,capacity=16,*,camera='secondary',confirm_frames=2,confirm_span_s=.05):
        if type(capacity) is not int or capacity<2:raise ValueError('invalid mission ledger capacity')
        if type(confirm_frames) is not int or confirm_frames<2:raise ValueError('invalid mission confirmation frames')
        if not numeric(confirm_span_s) or confirm_span_s<=0:raise ValueError('invalid mission confirmation span')
        if camera!='secondary':raise ValueError('task counting requires the secondary camera')
        self.capacity,self.camera=capacity,camera
        self.confirm_frames,self.confirm_span_s=confirm_frames,confirm_span_s
        self.records={};self.candidate=None
        self.overflow=False;self.first_overflow=None

    def _key(self,camera,identifier):
        if (camera!=self.camera or not isinstance(identifier,str)
                or not identifier.startswith(camera+'-') or not 1<=len(identifier)<=128):return None
        return camera,identifier

    def observe(self,camera,identifier,frame_id,captured_s,now,*,credible):
        key=self._key(camera,identifier)
        if (not credible or key is None or type(frame_id) is not int or frame_id<0
                or not numeric(captured_s) or not numeric(now) or not 0<=now-captured_s<=.25):
            self.candidate=None
            return False
        if key in self.records:
            self.candidate=None
            self.records[key]['last_observed_s']=captured_s
            return True
        previous=self.candidate
        if (previous is None or previous['key']!=key or captured_s-previous['last_s']>.25
                or captured_s<previous['last_s'] or frame_id<previous['frame_id']):
            previous={'key':key,'first_s':captured_s,'last_s':captured_s,'frame_id':frame_id,'frames':1,'qualified':False}
            self.candidate=previous
        elif frame_id!=previous['frame_id'] and captured_s>previous['last_s']:
            previous.update(last_s=captured_s,frame_id=frame_id,frames=previous['frames']+1)
        if (previous['frames']<self.confirm_frames
                or previous['last_s']-previous['first_s']<self.confirm_span_s-1e-9):return False
        if len(self.records)>=self.capacity:
            self.overflow=True
            if self.first_overflow is None:
                self.first_overflow={'camera':camera,'id':identifier,'at_s':captured_s}
            previous['qualified']=True
            return False
        self.records[key]={'camera':camera,'id':identifier,'state':'unresolved',
            'registered_s':captured_s,'first_observed_s':previous['first_s'],
            'last_observed_s':captured_s,'estimate_kind':None,'physical_tail_verified':False}
        self.candidate=None
        return True

    def registered(self,camera,identifier):
        return self._key(camera,identifier) in self.records

    def commit_estimate(self,camera,identifier,evidence,now):
        key=self._key(camera,identifier);entry=self.records.get(key)
        if (entry is None or entry['state']!='unresolved' or not isinstance(evidence,dict)
                or evidence.get('visual_estimate_ready') is not True
                or evidence.get('target_id')!=identifier or evidence.get('source_camera')!=camera
                or type(evidence.get('frame_id')) is not int or evidence['frame_id']<0
                or not numeric(evidence.get('captured_s')) or not numeric(now)
                or not 0<=now-evidence['captured_s']<=.25):return False
        entry.update(state='passed_estimate',estimate_kind='image_sequence_estimate',
            physical_tail_verified=False,estimated_pass_s=evidence.get('captured_s'),
            evidence_summary={key:evidence.get(key) for key in ('frame_id','captured_s',
                'gate_hits','exit_frames','reason','identity_revision','observation_deadline_s')})
        return True

    def completion_ready(self,required=2):
        return (not self.overflow and not any(row['state']=='unresolved' for row in self.records.values())
                and sum(row['state']=='passed_estimate' for row in self.records.values())>=required)

    def diagnostics(self):
        unresolved=[dict(row) for row in self.records.values() if row['state']=='unresolved']
        passed=[dict(row) for row in self.records.values() if row['state']=='passed_estimate']
        reasons=[]
        if self.overflow:reasons.append('task ledger capacity reached; unresolved identity cannot be discarded')
        if unresolved:reasons.append('registered task cones remain unresolved')
        if len(passed)<2:reasons.append('fewer than two registered visual pass estimates')
        return {'mission_camera':self.camera,'auxiliary_camera_policy':'guidance_only_no_task_registration_or_count',
            'mission_confirmation_frames':self.confirm_frames,'mission_confirmation_span_s':self.confirm_span_s,
            'mission_registered_count':len(self.records),'mission_unresolved':unresolved,
            'mission_passed_estimates':passed,'mission_overflow':self.overflow,
            'mission_first_overflow':dict(self.first_overflow) if self.first_overflow else None,
            'mission_completion_ready':self.completion_ready(),'mission_completion_blockers':reasons,
            'physical_tail_verified':False}
