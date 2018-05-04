class Event:
    def __init__(self, name='annoymous'):
        self.name = name

    def __str__(self):
        return '<%s.Event:%s>' % (__package__, self.name)

    def getname(self):
        return self.name


if __name__ == '__main__':
    ev = Event()
    print(ev)
    ev = Event('test event')
    print(ev)
