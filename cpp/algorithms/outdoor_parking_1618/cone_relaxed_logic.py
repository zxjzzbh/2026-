"""原版绕桶及正常回正 + 第二桶换号接续；软异常交回普通巡线。

The legacy --cone-mode relaxed selector now selects this replacement policy.
Detection, pass confirmation and normal ALIGN stay original.
"""
from cone_solo_logic import Avoider


class RelaxedAvoider(Avoider):
    def __init__(self, params=None):
        super().__init__(params)
        self.completed_targets = set()
        self.last_target_relink = None

    def _finish_avoid(self, boxes, mask, dt):
        completed = self.target_id if self.pass_confirmed else None
        result = super()._finish_avoid(boxes, mask, dt)
        result.completed_target_id = completed
        if completed is not None:
            self.completed_targets.add(completed)
        return result

    def halt(self, reason, boxes=None, mask=None):
        if self.fault:
            return super().halt(self.fault, boxes, mask)
        if (not self.p.protection_stop and reason in
                ('TARGET_UNCERTAIN', 'PASS_UNCONFIRMED', 'NO_AVOID_PROGRESS')):
            # Abandon uncertainty without inventing a passed-cone event.
            # Fusion selects ordinary lane steering, or stops for reacquisition.
            self.pass_confirmed = False
            result = self._finish_avoid(boxes or [], mask, 0.)
            result.handoff_turn_us = None
            result.reason = reason + '_LANE_FOLLOW'
            result.lane_follow_fallback = True
            return result
        return super().halt(reason, boxes, mask)

    def _after_tracking(self, boxes, captured_at):
        self.last_target_relink = None
        if (len(self.completed_targets) != 1 or self.state not in ('AVOID', 'PASS')
                or self.target_id is None or len(boxes) != 1):
            return
        old_id = self.target_id
        old_track = self.tracker.tracks.get(old_id)
        if old_track is not None and old_track['miss'] == 0:
            return
        candidate = boxes[0]
        new_id = candidate['id']
        if new_id == old_id or new_id in self.completed_targets:
            return
        new_track = self.tracker.tracks[new_id]
        # Continue the second target without restarting its direction or timing.
        self.target_id = new_id
        for ids in (self.active, self.pass_targets, self.exit_evidence):
            if old_id in ids:
                ids.remove(old_id)
                ids.add(new_id)
        # Original AVOID can remove an unseen target before starting PASS.
        # A visible replacement must rejoin the pass check, not leave an
        # empty target set that could confirm passage while it is still seen.
        self.active.add(new_id)
        if self.state == 'PASS':
            self.pass_targets.add(new_id)
        if old_id in self.clear_hits:
            self.clear_hits[new_id] = self.clear_hits.pop(old_id)
        new_track['confirmed'] = True
        candidate['confirmed'] = True
        self.tracker.tracks.pop(old_id, None)
        self.last_target_relink = {'from': old_id, 'to': new_id}
