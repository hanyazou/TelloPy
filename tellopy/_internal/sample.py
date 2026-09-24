class Sample(object):
    """Common base for a single timestamped observation.

    event_time is the best available host-time estimate of when the
    thing being recorded actually happened. tick is the raw device
    counter this observation carries, if any -- kept unconverted since
    it is more precise than any host-time estimate derived from it.
    Not every kind of Sample has a tick (e.g. a command we ourselves
    generate has no device counter at all), so it defaults to None.

    recv_time is the host time the packet carrying this observation
    arrived, or None if that hasn't happened (yet).
    """
    def __init__(self, event_time, tick=None, recv_time=None):
        self.event_time = event_time
        self.tick = tick
        self.recv_time = recv_time


class CommandSample(Sample):
    """One outgoing command and, if and when it arrives, the response
    that completes it.

    event_time is the moment the command was sent (there is no device
    tick for something we generate ourselves, so tick is always None
    here). recv_time/ack_payload stay None until a matching response
    arrives, and never fill in at all for commands that have no ack
    mechanism to begin with (e.g. the continuous stick/RC command).
    """
    def __init__(self, cmd, seq, name, send_time, payload):
        super(CommandSample, self).__init__(event_time=send_time, tick=None)
        self.cmd = cmd
        self.seq = seq
        self.name = name
        self.send_time = send_time
        self.payload = payload
        self.ack_payload = None

    def mark_acked(self, recv_time, ack_payload):
        self.recv_time = recv_time
        self.ack_payload = ack_payload

    @property
    def acked(self):
        return self.recv_time is not None

    @property
    def rtt(self):
        if self.recv_time is None:
            return None
        return self.recv_time - self.send_time

    def __str__(self):
        if self.acked:
            return '%s (cmd=0x%04x seq=0x%04x rtt=%.1fms)' % (
                self.name, self.cmd, self.seq, self.rtt * 1000.0)
        return '%s (cmd=0x%04x seq=0x%04x, unacked)' % (self.name, self.cmd, self.seq)


class StickSample(Sample):
    """A stick command as it went out: where the four axes (each -1..1)
    and the fast-mode switch were when the packet was sent.

    event_time is the moment of sending, on the same clock as
    CommandSample.send_time. The stick command is sent continuously and the
    drone never answers it, so there is nothing to wait for: it is
    published right after sending, with no tick and no recv_time.
    """
    def __init__(self, send_time, roll, pitch, throttle, yaw, fast_mode):
        super(StickSample, self).__init__(event_time=send_time, tick=None)
        self.send_time = send_time
        self.roll = roll
        self.pitch = pitch
        self.throttle = throttle
        self.yaw = yaw
        self.fast_mode = fast_mode

    def __str__(self):
        return 'stick (roll=%.2f pitch=%.2f throttle=%.2f yaw=%.2f%s)' % (
            self.roll, self.pitch, self.throttle, self.yaw, ' fast' if self.fast_mode else '')
