"""Browser supervision is separate from the short, local actuator lease.

Only authenticated task adapters call this helper. Read-only status/image
requests cannot renew it; duplicates cannot revive a cancelled or expired run.
"""

PAGE_LEASE_S = 3.0


class PageHeartbeat:
    def __init__(self):
        self.sequence = -1
        self.last_received_s = None
        self.received = 0
        self.maximum_gap_s = 0.
        self.deadline = 0.

    def reset(self, now):
        self.sequence = -1
        self.last_received_s = now
        self.received = 0
        self.maximum_gap_s = 0.
        self.deadline = now + PAGE_LEASE_S
        return self.deadline

    def renew(self, now):
        # Serialized task commands may carry an older receipt timestamp.
        now = max(now, self.last_received_s if self.last_received_s is not None else now)
        if self.last_received_s is not None:
            self.maximum_gap_s = max(self.maximum_gap_s, now-self.last_received_s)
        self.last_received_s = now
        self.received += 1
        self.deadline = now + PAGE_LEASE_S
        return self.deadline

    def accept(self, payload, now, deadline):
        if now > deadline:
            raise ValueError('控制页连续 3 秒未收到心跳，请重新准备')
        sequence = payload.get('heartbeat_sequence')
        if type(sequence) is not int or sequence < 0:
            raise ValueError('invalid heartbeat sequence')
        accepted = sequence > self.sequence
        if accepted:
            self.sequence = sequence
            self.renew(now)
        return {'heartbeat_ack': True, 'accepted': accepted,
                'heartbeat_sequence': sequence, 'server_received_s': now,
                'page_lease_s': PAGE_LEASE_S}

    def status(self, now):
        return {'lease_s': PAGE_LEASE_S, 'last_received_s': self.last_received_s,
                'age_s': None if self.last_received_s is None else max(0., now-self.last_received_s),
                'received': self.received, 'maximum_receive_gap_s': self.maximum_gap_s}
