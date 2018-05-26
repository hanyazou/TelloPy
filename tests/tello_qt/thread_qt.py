import sys
import time
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import wifi


class WifiThread(QThread):
    updated = pyqtSignal()

    def __init__(self):
        QThread.__init__(self)
        self.wifi_device_name = 'unknown'
        self.wifi_status = 'unknown'
        self.wifi_active = False
        self.wifi_ssid = 'unknown'

    def __del__(self):
        self.wait()

    def update(self):
        self.wifi_device_name = wifi.get_device_name()
        self.wifi_status = wifi.get_status()
        self.wifi_active = True if self.wifi_status == 'active' else False
        self.wifi_ssid = (wifi.get_ssid() if self.wifi_active else '-')
        self.updated.emit()

    def run(self):
        self.update()
        while True:
            wifi.wait()
            self.update()
            self.sleep(1)
