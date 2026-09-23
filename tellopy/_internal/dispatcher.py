import inspect

from . import event


class signal(object):
    All = event.Event('*')


signals = {}

# receiver -> the keyword argument names it accepts, or None if it takes
# arbitrary ones (**kwargs) or can't be inspected. Events carry more
# keyword arguments over time (e.g. recv_time); this lets a receiver
# written against the older, narrower set -- def handler(event, sender,
# data) -- keep working, receiving only what it asks for.
accepted_kwargs = {}


def _accepted_kwargs(receiver):
    try:
        params = inspect.signature(receiver).parameters.values()
    except (TypeError, ValueError):
        return None
    names = set()
    for param in params:
        if param.kind is param.VAR_KEYWORD:
            return None
        if param.kind in (param.POSITIONAL_OR_KEYWORD, param.KEYWORD_ONLY):
            names.add(param.name)
    return names


def connect(receiver, sig=signal.All):
    if sig in signals:
        receivers = signals[sig]
    else:
        receivers = signals[sig] = []
    receivers.append(receiver)
    accepted_kwargs[receiver] = _accepted_kwargs(receiver)


def disconnect(receiver, sig=signal.All):
    if sig is signal.All:
        for sig in signals:
            if receiver in signals[sig]:
                signals[sig].remove(receiver)
    elif sig in signals:
        if receiver in signals[sig]:
            signals[sig].remove(receiver)
    if not any(receiver in receivers for receivers in signals.values()):
        accepted_kwargs.pop(receiver, None)


def send(sig, **named):
    if sig in signals:
        receivers = signals[sig] + signals[signal.All]
    else:
        receivers = signals[signal.All]
    for receiver in receivers:
        accepted = accepted_kwargs.get(receiver)
        if accepted is None:
            receiver(event=sig, **named)
        else:
            receiver(event=sig, **dict((k, v) for k, v in named.items() if k in accepted))


if __name__ == '__main__':
    def handler0(event, sender, **args):
        recvs.append(0)
        print('handler0: event=%s sender=%s' % (str(event), str(sender)))
        print(args)

    def handler1(event, sender, **args):
        recvs.append(1)
        print('handler1: event=%s sender=%s' % (str(event), str(sender)))
        print(args)

    test_signal0 = event.Event('test signal0')
    test_signal1 = event.Event('test signal1')
    connect(handler0, signal.All)
    connect(handler1, test_signal0)

    recvs = []
    send(test_signal0, sender=None)
    assert len(recvs) == 2 and 0 in recvs and 1 in recvs

    recvs = []
    send(test_signal1, sender=None, data='test data')
    assert len(recvs) == 1 and 0 in recvs

    disconnect(handler1)

    recvs = []
    send(test_signal0, sender=None, arg0=0, arg1=1, arg2=2)
    assert len(recvs) == 1 and 0 in recvs

    # A receiver only gets the keyword arguments it declares (unless it
    # takes **kwargs), so ones written before an argument existed still work.
    got = {}

    def narrow(event, sender, data):
        got['narrow'] = (data,)

    def wide(event, sender, data, recv_time):
        got['wide'] = (data, recv_time)

    def catch_all(event, **args):
        got['catch_all'] = sorted(args)

    for handler in (narrow, wide, catch_all):
        connect(handler, test_signal1)
    send(test_signal1, sender=None, data='d', recv_time=1.5)
    assert got == {'narrow': ('d',), 'wide': ('d', 1.5),
                   'catch_all': ['data', 'recv_time', 'sender']}, got

    for handler in (narrow, wide, catch_all):
        disconnect(handler, test_signal1)
    assert narrow not in accepted_kwargs
