#!/usr/bin/env python
#
# Main window for OAC monitor 
#
# @author: Zhenhua Cai, czhedu@gmail.com 
# @date:   2011-03-30
#
# @note: I borrowed some code from  
# http://matplotlib.sourceforge.net/examples/user_interfaces/embedding_in_qt4.html
#

import sys
import zmq

from PyQt4 import QtGui, QtCore

from monitor_widget import MonitorWidget, MonitorThread

program_name = "OAC monitor"
updated_date = "2011-03-30"

class MonitorMainWindow(QtGui.QMainWindow):
    """ Main window class """
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle("OAC monitor")

        # Menu
        self.file_menu = QtGui.QMenu("&File", self)
        self.file_menu.addAction("&Quit", self.file_quit, QtCore.Qt.CTRL+QtCore.Qt.Key_Q)
      
        self.help_menu = QtGui.QMenu("&Help", self)
        self.help_menu.addAction("&About", self.help_about)

        self.menuBar().addMenu(self.file_menu)
        self.menuBar().addSeparator()
        self.menuBar().addMenu(self.help_menu)

        # Create and layout all sorts of widgets, each is responsible for a mind agent 
        self.add_widgets()

        # Status bar
        self.statusBar().showMessage("OAC monitor", 2000)

    def add_widgets(self):
        self.zmq_context = zmq.Context()
        self.main_widget = QtGui.QWidget(self)
        grid_layout = QtGui.QGridLayout(self.main_widget)

        # Add PsiModulatorUpdaterAgentMonitor
        self.PsiModulatorUpdaterAgentMonitor = MonitorThread(self.zmq_context,
                             "tcp://127.0.0.1:18002", 
                             "PsiModulatorUpdaterAgent", 
                             self.main_widget, width=5, height=4, dpi=100
                            )
        grid_layout.addWidget(self.PsiModulatorUpdaterAgentMonitor.widget, 0, 0)

        # Add PsiFeelingUpdaterAgentMonitor
        self.PsiFeelingUpdaterAgentMonitor = MonitorThread(self.zmq_context,
                             "tcp://127.0.0.1:18002", 
                             "PsiFeelingUpdaterAgent", 
                             self.main_widget, width=5, height=4, dpi=100
                            )
        grid_layout.addWidget(self.PsiFeelingUpdaterAgentMonitor.widget, 0, 1)

        # Add PsiDemandUpdaterAgentMonitor
        self.PsiDemandUpdaterAgentMonitor = MonitorThread(self.zmq_context,
                             "tcp://127.0.0.1:18002", 
                             "PsiDemandUpdaterAgent", 
                             self.main_widget, width=5, height=4, dpi=100
                            )
        grid_layout.addWidget(self.PsiDemandUpdaterAgentMonitor.widget, 1, 0)

        self.main_widget.setFocus()
        self.setCentralWidget(self.main_widget)

        # Don't forget start the threads of all the monitors,
        # otherwise you will get nothing!
        self.PsiModulatorUpdaterAgentMonitor.start()
        self.PsiFeelingUpdaterAgentMonitor.start()
        self.PsiDemandUpdaterAgentMonitor.start()

    def file_quit(self):
        self.close()

    def help_about(self):
        QtGui.QMessageBox.about(self, "About %s" % program_name, """This is a simple monitor which facilities watching internal states and parameters within OAC such as modulators, emotional states etc.""")

# Create and run the application and the main window
app = QtGui.QApplication(sys.argv)

main_window = MonitorMainWindow()
main_window.setWindowTitle("OAC monitor")
main_window.show()
sys.exit(app.exec_())
