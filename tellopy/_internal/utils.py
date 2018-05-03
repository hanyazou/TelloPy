import sys
import traceback


def le16(val):
    return (val & 0xff), ((val >> 8) & 0xff)


def int16(val0, val1):
    return (val0 & 0xff) | ((val1 & 0xff) << 8)


def byte_to_hexstring(buf):
    if isinstance(buf, str):
        return ''.join(["%02x " % ord(x) for x in buf]).strip()

    return ''.join(["%02x " % ord(chr(x)) for x in buf]).strip()


def show_exception(ex):
    exc_type, exc_value, exc_traceback = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_traceback)
