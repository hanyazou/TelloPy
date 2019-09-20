import datetime
import threading

LOG_ERROR = 0
LOG_WARN = 1
LOG_INFO = 2
LOG_DEBUG = 3
LOG_ALL = 99


class Logger(object): #Object inheritence for child 'RospyLogger' (Jordy)
    def __init__(self, header=''):
        self.log_level = LOG_INFO
        self.header_string = header
        self.lock = threading.Lock()

    def header(self):
        now = datetime.datetime.now()
        ts = ("%02d:%02d:%02d.%03d" % (now.hour, now.minute, now.second, now.microsecond/1000))
        return "%s: %s" % (self.header_string, ts)

    def set_level(self, level):
        self.log_level = level

    def output(self, msg):
        self.lock.acquire()
        print(msg)
        self.lock.release()

    def error(self, str):
        if self.log_level < LOG_ERROR:
            return
        self.output("%s: Error: %s" % (self.header(), str))

    def warn(self, str):
        if self.log_level < LOG_WARN:
            return
        self.output("%s:  Warn: %s" % (self.header(), str))

    def info(self, str):
        if self.log_level < LOG_INFO:
            return
        self.output("%s:  Info: %s" % (self.header(), str))

    def debug(self, str):
        if self.log_level < LOG_DEBUG:
            return
        self.output("%s: Debug: %s" % (self.header(), str))

if __name__ == '__main__':
    log = Logger('test')
    log.error('This is an error message')
    log.warn('This is a warning message')
    log.info('This is an info message')
    log.debug('This should ** NOT **  be displayed')
    log.set_level(LOG_ALL)
    log.debug('This is a debug message')
