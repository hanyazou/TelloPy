from __future__ import division, print_function, absolute_import
 
import sys
from PyQt5.QtCore import Qt
from PyQt5 import uic, QtWidgets
from thread_qt import WifiThread
 
qtCreatorFile = "mainwindow.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)
 
class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)

        self.status_label.setAutoFillBackground(True)

        self.status_palette_connected = self.status_label.palette()
        self.status_palette_connected.setColor(self.status_label.backgroundRole(), Qt.green)
        self.status_palette_connected.setColor(self.status_label.foregroundRole(), Qt.black)

        self.status_palette_disconnected = self.status_label.palette()
        self.status_palette_disconnected.setColor(self.status_label.backgroundRole(), Qt.yellow)
        self.status_palette_disconnected.setColor(self.status_label.foregroundRole(), Qt.black)

        self.status_palette_paused = self.status_label.palette()
        self.status_palette_paused.setColor(self.status_label.backgroundRole(), Qt.gray)
        self.status_palette_paused.setColor(self.status_label.foregroundRole(), Qt.black)

        self.wifi_thread = WifiThread()
        self.wifi_thread.updated.connect(self.handler)
        self.wifi_thread.start()

    def handler(self):
        if self.wifi_thread.wifi_active:
            text = self.wifi_thread.wifi_ssid
            palette = self.status_palette_connected
        else:
            text = 'no connection'
            palette = self.status_palette_disconnected

        self.status_label.setPalette(palette);
        self.status_label.setText(text)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())
