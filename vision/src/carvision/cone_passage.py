"""Same-slice visual passage estimates, never physical tail-clearance proof.

Two current same-slice clear observations establish a visual anchor, without
waiting for the near exit band. Later same-slice contradictions invalidate it.
Only an additional current pixel exit sequence may complete the estimate. The
fixed five-second observation window can reject an estimate, never create one.
"""
from .brake_parking import numeric
from .cone_evidence import paired_lane
from .cone_response import current_visual


OBSERVATION_S=5.
ANCHOR_FRAMES=2
ANCHOR_SPAN_S=.05
MIN_FRAMES=3
MIN_SPAN_S=.15
EXIT_BOTTOM=.85
EXIT_SIDE=.75
EXIT_PROGRESS=.03


class PassageEvidence:
    def __init__(self,settings):
        geometry=settings.get('geometry',{});options=settings.get('low_speed_test') or {}
        width,lane,margin=(geometry.get('vehicle_width_m'),geometry.get('lane_width_m'),options.get('clearance_m'))
        self.vehicle_half=(width/2+margin)/lane if all(numeric(v) for v in (width,lane,margin)) and width>0 and lane>0 and margin>=0 else None
        size=settings.get('image_size')
        self.image_size=tuple(size) if isinstance(size,(list,tuple)) and len(size)==2 and all(type(v) is int and v>0 for v in size) else None
        self.reset()

    def reset(self,reason='new_target'):
        self.identifier=self.camera=self.side=None
        self.revision=0;self.frame_id=self.captured_s=None
        self.gate_hits=0;self.near_seen=False;self.min_y=self.last_y=self.last_ground_s=None
        self.gate_first_s=None;self.candidate=False;self.candidate_s=self.candidate_frame=None
        self.anchor_hits=0;self.anchor_first_s=self.anchor_last_s=self.anchor_established_s=None
        self.anchor_rejection=None;self.legacy_gate_candidate=False;self.expired=False
        self.ready=False;self.exit_samples=[];self.phase='observing';self.reason=reason
        self.fresh_evidence=False;self.recovery_frozen=False

    def diagnostics(self,now):
        deadline=None if self.candidate_s is None else self.candidate_s+OBSERVATION_S
        return {'target_id':self.identifier,'source_camera':self.camera,
            'frame_id':self.frame_id,'captured_s':self.captured_s,
            'gate_hits':self.gate_hits,'near_seen':self.near_seen,
            'visual_pass_candidate':self.candidate,'visual_estimate_ready':self.ready,
            'anchor_valid':self.candidate,'anchor_hits':self.anchor_hits,
            'anchor_rejection':self.anchor_rejection,'anchor_established_s':self.anchor_established_s,
            'anchor_frame_id':self.candidate_frame,'legacy_gate_candidate':self.legacy_gate_candidate,
            'tail_verified':False,'physical_tail_verified':False,
            'evidence_kind':'image_sequence_estimate','phase':self.phase,'reason':self.reason,
            'observation_deadline_s':deadline,
            'observation_remaining_s':max(0.,deadline-now) if deadline is not None else 0.,
            'exit_frames':len(self.exit_samples),'fresh_evidence':self.fresh_evidence,
            'identity_revision':self.revision,'recovery_candidate_frozen':self.recovery_frozen}

    def _clear_partial(self):
        self.gate_hits=0;self.gate_first_s=None
        self.min_y=self.last_y=self.last_ground_s=None;self.near_seen=False

    def _recovery_valid(self,entry,nav):
        event=entry.get('identity_recovery') or {}
        if not isinstance(event,dict):return False
        frames=event.get('evidence_frame_ids',[])
        anchor=event.get('anchor_frame_id')
        return bool(entry.get('identity_recovered') is True and isinstance(event,dict)
            and event.get('source_camera')==self.camera
            and event.get('recovered_s')==nav.get('captured_s')
            and event.get('ground_evidence') is False
            and isinstance(frames,list) and len(frames)==2
            and all(type(fid) is int for fid in frames) and frames[0]<frames[1]==nav.get('frame_id')
            and type(anchor) is int and self.candidate_frame is not None
            and self.candidate_frame<=anchor<frames[0])

    def _same_slice(self,target,nav):
        proof=target.get('pass_evidence') or {}
        if (not isinstance(proof,dict) or not paired_lane(nav)
                or target.get('near_field_continuation') or target.get('provisional')
                or proof.get('within_observed_band') is not True
                or proof.get('target_id')!=self.identifier
                or proof.get('camera')!=self.camera
                or proof.get('frame_id')!=nav.get('frame_id')
                or proof.get('captured_s')!=nav.get('captured_s')):return None
        reference,cone,half,y=(proof.get(k) for k in ('reference_fraction_at_foot',
            'cone_fraction_at_foot','cone_half_width_fraction','foot_y_fraction'))
        if (not all(numeric(v) for v in (reference,cone,half,y))
                or not 0<=reference<=1 or not 0<=cone<=1 or not 0<half<=1
                or not 0<=y<=1 or target.get('fraction')!=cone
                or target.get('half_width')!=half or target.get('y_fraction')!=y):return None
        return reference,cone,half,y

    def _clearance(self,proof):
        reference,cone,half,_=proof
        gap=(cone-reference) if self.side=='left' else (reference-cone)
        return self.vehicle_half is not None and gap>self.vehicle_half+half

    def _reject_anchor(self,t,y):
        self.candidate=self.ready=False;self.exit_samples=[]
        self.anchor_hits=0;self.anchor_first_s=self.anchor_last_s=self.anchor_established_s=None
        self.anchor_rejection='current_same_slice_clearance_insufficient'
        self.gate_hits=0;self.gate_first_s=None;self.legacy_gate_candidate=False
        self.last_ground_s=t;self.last_y=y;self.phase='observing'
        self.reason=self.anchor_rejection

    def _observe_gate(self,target,proof,t):
        if self.last_ground_s is not None and t-self.last_ground_s>.25:
            self._clear_partial()
            self.legacy_gate_candidate=False
            if not self.candidate:self.anchor_hits=0;self.anchor_first_s=None
        reference,cone,half,y=proof
        self.near_seen|=target.get('near') is True
        self.min_y=y if self.min_y is None else min(self.min_y,y)
        clear=self._clearance(proof)
        if not clear:
            # A previous good anchor must never mask a later same-row conflict.
            self._reject_anchor(t,y)
            return
        self.anchor_rejection=None
        if not self.candidate:
            if self.anchor_last_s is None or t-self.anchor_last_s>.25:
                self.anchor_hits=0;self.anchor_first_s=t
            self.anchor_hits+=1;self.anchor_last_s=t
            if (self.anchor_hits>=ANCHOR_FRAMES and t-self.anchor_first_s>=ANCHOR_SPAN_S-1e-9):
                self.candidate=True;self.candidate_frame=self.frame_id;self.anchor_established_s=t
                # Rebuilding an invalidated anchor cannot extend this episode.
                if self.candidate_s is None:self.candidate_s=t
                self.phase='candidate';self.reason='early_same_foot_row_clearance_anchor'
        qualifies=(clear
            and self.near_seen and target.get('exit_band') is True and y-self.min_y>=.06
            and (self.last_y is None or y>=self.last_y-.02))
        if qualifies:
            if self.gate_hits==0:self.gate_first_s=t
            self.gate_hits+=1
            self.reason='collecting_current_same_foot_clearance'
        else:self.gate_hits=0;self.gate_first_s=None
        if not qualifies:self.reason=('same_foot_anchor_waiting_for_current_pixel_exit' if self.candidate
                                      else 'same_foot_clearance_or_progress_insufficient')
        self.last_ground_s=t;self.last_y=y
        if (self.gate_hits>=MIN_FRAMES and t-self.gate_first_s>=MIN_SPAN_S-1e-9):
            self.legacy_gate_candidate=True

    def _observe_exit(self,visual,t):
        x,y,half=(visual.get(k) for k in ('image_fraction','y_fraction','half_image_width'))
        reference=visual.get('image_reference_fraction')
        box=visual.get('bbox_xyxy')
        if (not numeric(y) or not 0<=y<=1 or self.image_size is None
                or not isinstance(box,(list,tuple)) or len(box)!=4
                or not all(numeric(value) for value in box)):
            self.exit_samples=[];return
        w,h=self.image_size;x1,y1,x2,y2=box
        if (not 0<=x1<x2<=w or not 0<=y1<y2<=h
                or not x1<=x*w<=x2 or not y1<=y*h<=y2
                or abs(half-(x2-x1)/(2*w))>1e-9):
            self.exit_samples=[];return
        outer=(x-half>=EXIT_SIDE and x>reference) if self.side=='left' else (x+half<=1-EXIT_SIDE and x<reference)
        if y<EXIT_BOTTOM or not outer:
            self.exit_samples=[];return
        self.near_seen=True
        previous=self.exit_samples[-1] if self.exit_samples else None
        direction=1 if self.side=='left' else -1
        if previous and (t-previous[0]>.25 or y<previous[2]-.02 or direction*(x-previous[1])<-.02):
            self.exit_samples=[]
        self.exit_samples.append((t,x,y));self.exit_samples=self.exit_samples[-MIN_FRAMES:]
        first=self.exit_samples[0]
        progressed=(y-first[2]>=EXIT_PROGRESS or direction*(x-first[1])>=EXIT_PROGRESS)
        if (len(self.exit_samples)>=MIN_FRAMES and t-first[0]>=MIN_SPAN_S-1e-9 and progressed):
            self.ready=True;self.phase='visual_estimate';self.reason='current_same_id_bottom_side_exit_sequence'
        else:self.phase='exit_observation';self.reason='awaiting_current_pixel_exit_progress'

    def observe(self,target,nav,side,now):
        target=target if isinstance(target,dict) else {}
        nav=nav if isinstance(nav,dict) else {}
        identifier=target.get('id',self.identifier);camera=nav.get('source_camera')
        self.fresh_evidence=False;self.recovery_frozen=False
        if (identifier,camera,side)!=(self.identifier,self.camera,self.side):
            self.reset('target_camera_or_side_changed')
            self.identifier,self.camera,self.side=identifier,camera,side
        t,fid=nav.get('captured_s'),nav.get('frame_id')
        if not identifier or side not in ('left','right') or not numeric(t) or type(fid) is not int or not numeric(now) or not 0<=now-t<=.25:
            self.reason='no_fresh_bound_target';return self.diagnostics(now if numeric(now) else 0.)
        if self.expired or self.candidate_s is not None and now>=self.candidate_s+OBSERVATION_S-1e-9:
            self.candidate=False;self.ready=False;self._clear_partial();self.exit_samples=[]
            self.anchor_hits=0;self.anchor_rejection='observation_deadline_expired';self.expired=True
            self.phase='unverified';self.reason='observation_deadline_without_exit_evidence'
            return self.diagnostics(now)
        if self.frame_id is not None and (fid<=self.frame_id or t<=self.captured_s):
            self.reason='duplicate_or_old_frame';return self.diagnostics(now)
        visual=current_visual(nav,identifier)
        entry=visual or target;revision=entry.get('identity_revision',self.revision)
        if type(revision) is not int or revision<0:revision=-1
        if revision!=self.revision:
            if self.candidate and revision==self.revision+1 and self._recovery_valid(entry,nav):
                self.ready=False;self.exit_samples=[];self.recovery_frozen=True;self.phase='candidate'
                self.reason='unique_recovery_freezes_candidate_not_exit_evidence'
                # Recovery may retain earlier proof, but cannot hide a current
                # contradiction or add a new ground gate on this recovery frame.
                proof=self._same_slice(target,nav)
                if proof and not self._clearance(proof):self._reject_anchor(t,proof[3])
            else:
                episode_start=self.candidate_s
                self.reset('identity_revision_conflict')
                self.identifier,self.camera,self.side=identifier,camera,side
                # Re-identification may discard proof, never buy this same
                # target/camera/side a new five-second observation budget.
                self.candidate_s=episode_start
                self.anchor_rejection='identity_revision_conflict'
            self.revision=revision;self.frame_id,self.captured_s=fid,t
            return self.diagnostics(now)
        self.frame_id,self.captured_s=fid,t;self.fresh_evidence=True
        self.ready=False
        # Current same-slice evidence always revalidates the anchor, including
        # while following pixels; a bad late ground frame is an explicit veto.
        proof=self._same_slice(target,nav)
        if proof:self._observe_gate(target,proof,t)
        else:
            self.gate_hits=0;self.gate_first_s=None;self.legacy_gate_candidate=False
            if not self.candidate:
                self.anchor_hits=0;self.anchor_first_s=self.anchor_last_s=None
                self.reason='no_current_same_foot_row_ground_evidence'
        if self.candidate and visual and t>self.candidate_s:
            self._observe_exit(visual,t)
        elif self.candidate:
            self.exit_samples=[];self.reason='candidate_waits_for_current_pixel_exit'
        return self.diagnostics(now)
