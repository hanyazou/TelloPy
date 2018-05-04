class State(object):
    def __init__(self, name='annoymous'):
        self.name = name

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return '%s::%s' % (self.__class__.__name__, self.name)

    def getname(self):
        return self.name


if __name__ == '__main__':
    st = State()
    print(st)
    st = State('test state')
    print(st)
