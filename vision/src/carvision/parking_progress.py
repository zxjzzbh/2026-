"""Keep confirmed parking progress without treating lost pixels as crossing."""


class ParkingProgress:
    ORDER=('seek','approach','align_entry','cross_entry','inside_estimate')

    def __init__(self):
        self.stage='seek';self.pending=None;self.since=None;self.frames=0;self.last=None

    @property
    def entered(self):return self.ORDER.index(self.stage)>=self.ORDER.index('cross_entry')

    def observe(self,candidate,t):
        if self.last is not None and t<=self.last:return self.stage
        if self.last is not None and t-self.last>.5:self.missing()
        self.last=t
        if self.ORDER.index(candidate)<=self.ORDER.index(self.stage):
            self.pending=self.since=None;self.frames=0
            return self.stage
        if candidate not in ('cross_entry','inside_estimate'):
            self.stage=candidate;return self.stage
        if candidate!=self.pending:self.pending=candidate;self.since=t;self.frames=0
        self.frames+=1
        if self.frames>=2 and t-self.since>=.08-1e-9:
            self.stage=candidate;self.pending=self.since=None;self.frames=0
        return self.stage

    def missing(self):
        # Progress is retained; current position and completion are not.
        self.pending=self.since=None;self.frames=0
