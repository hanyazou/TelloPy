from . import event


class signal(object):
    All = event.Event('*')


signals = {}


def connect(receiver, sig=signal.All):
    if sig in signals:
        receivers = signals[sig]
    else:
        receivers = signals[sig] = []
    receivers.append(receiver)


def disconnect(receiver, sig=signal.All):
    if sig is signal.All:
        for sig in signals:
            if receiver in signals[sig]:
                signals[sig].remove(receiver)
    elif sig in signals:
        if receiver in signals[sig]:
            signals[sig].remove(receiver)


def send(sig, **named):
    if sig in signals:
        receivers = signals[sig] + signals[signal.All]
    else:
        receivers = signals[signal.All]
    for receiver in receivers:
        receiver(event=sig, **named)


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
