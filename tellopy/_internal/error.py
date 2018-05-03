"""Error types for tellopy"""


class TelloError(Exception):
    """Base class for all Tello errors"""

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg

    def __repr__(self):
        return self.msg
