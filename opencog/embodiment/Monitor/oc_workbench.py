# -*- coding: utf-8 -*-

# An elegant chrome-like workbench UI for OpenCog.
#
# @author Troy Huang <huangdeheng@gmail.com>
# @date 2011-03-30
#



import sys
import zmq

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from monitor_widget import *

class OCMonitorPanel(QFrame):
    """
    OCMonitorPanel is the container of MonitorWidgets.
    View of MonitorWidgets can be changed through this panel.
    """
    def __init__(self, parent=None, columnNum=2):
        super(OCMonitorPanel, self).__init__(parent)

        self.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding, True))
        
        self.monitorList = []
        self.columnNum = columnNum

        self.soloist = None
        self.soloistOldSize = None
        self.zmqUrl = None

        gridLayout = QGridLayout(self)

        self.setLayout(gridLayout)

    def addMonitors(self, monitors):
        oldCount = len(self.monitorList)
        self.monitorList.extend(monitors)
        for i, monitor in enumerate(monitors):
            row = (oldCount + i) / self.columnNum
            col = (oldCount + i) % self.columnNum
            self.layout().addWidget(monitor.widget, row, col)
            self.layout().setRowStretch(row, 0)
            monitor.widget.clicked.connect(self.showSoloView)

    def createMonitors(self, zmqUrl=""):
        if zmqUrl == "":
            warning = QMessageBox.warning(self, "Warning",
                    "Please input a valid zmq url.")
            return
        if not self.zmqUrl or self.zmqUrl != zmqUrl:
            if len(self.monitorList) > 0 and self.zmqUrl != zmqUrl:
                del self.monitorList[:]
                self.updateLayout()
            self.zmqUrl = zmqUrl
        else:
            pass
        self.zmq_context = zmq.Context()
        
        monitor1 = MonitorThread(self.zmq_context,
                             str(self.zmqUrl),
                             "PsiModulatorUpdaterAgent", 
                             self)
        monitor2 = MonitorThread(self.zmq_context,
                             str(self.zmqUrl), 
                             "PsiFeelingUpdaterAgent", 
                             self)
        monitor3 = MonitorThread(self.zmq_context,
                             str(self.zmqUrl), 
                             "PsiDemandUpdaterAgent", 
                             self)
        self.addMonitors([monitor1, \
                    monitor2, \
                    monitor3]) 
        self.startMonitor()

    def startMonitor(self):
        for monitor in self.monitorList:
            monitor.start()

    def showSoloView(self):
        sender = self.sender()

        if self.soloist != sender:
            self.soloist = sender
            self.soloistOldSize = QSize(sender.size())
        else:
            return

        for monitor in self.monitorList:
            if sender != monitor.widget:
                monitor.widget.hide()
        sender.update()
        sender.updateGeometry()

        #self.updateSoloView()

    def updateSoloView(self):
        """
        Update solo view when there's a resize event
        """
        rect = QRect(0, 0, self.geometry().width(), self.geometry().height())
        self.soloist.setGeometry(rect)

    def showGroupView(self):
        self.soloist.resize(self.soloistOldSize)
        self.soloist = None
        for monitor in self.monitorList:
            monitor.widget.show()

    def heightForWidth(self, width):
        return 0.9 * width

    def sizeHint(self):
        return QSize(self.geometry().width(), self.heightForWidth(self.geometry().width()))
    

class OCMonitorTabView(QWidget):
    """
    """
    def __init__(self, parent=None):
        super(OCMonitorTabView, self).__init__(parent)

        # Initialize a navigation bar
        self.navigationBar = QToolBar(self)
        
        # Initialize a monitor panel
        self.monitorPanel = OCMonitorPanel(self)
        
        self.createNavigationBar()

        tabLayout = QVBoxLayout()
        tabLayout.addWidget(self.navigationBar, 0, Qt.AlignTop)
        tabLayout.addWidget(self.monitorPanel, 1, Qt.AlignTop)
        self.setLayout(tabLayout)


    def createNavigationBar(self):
        # Back action: go back to group monitors view.
        self.actionBack = QAction(self)
        self.actionBack.setIcon(QIcon("images/backward.png"))
        self.actionBack.setObjectName("actionBack")
        self.actionBack.triggered.connect(\
                self.monitorPanel.showGroupView)

        # Forward action: go forward to previous monitor panel view.
        self.actionForward = QAction(self)
        self.actionForward.setIcon(QIcon("images/forward.png"))
        self.actionForward.setObjectName("actionForward")

        # Start action: start a monitor instance.
        self.actionStart = QAction(self)
        self.actionStart.setIcon(QIcon("images/start.png"))
        self.actionStart.setObjectName("actionStart")

        # Add a navigation bar that contains above actions.
        self.navigationBar.setOrientation(Qt.Horizontal)

        self.navigationBar.setObjectName("navigationBar")
        self.navigationBar.setMovable(False)
        self.navigationBar.setFloatable(False)
        
        self.navigationBar.addAction(self.actionBack)
        self.navigationBar.addAction(self.actionForward)
        self.navigationBar.addAction(self.actionStart)

        # Add an address bar.
        self.addressEdit = QLineEdit(self.navigationBar)
        self.navigationBar.insertWidget(self.actionStart, self.addressEdit)
        self.addressEdit.setText("tcp://127.0.0.1:18002")
        self.actionStart.triggered.connect(self.createMonitors)

    def createMonitors(self):
        self.monitorPanel.createMonitors(self.addressEdit.text())

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Return:
            self.createMonitors()
    

class OCMonitorTabWidget(QTabWidget):
    def __init__(self):
        super(OCMonitorTabWidget, self).__init__(None)
        
        # Create an "add tab" button.
        self.addTabButton = QPushButton()
        self.addTabButton.setIcon(QIcon("images/add.png"))
        # Connect clicked signal to add new tab slot.
        self.addTabButton.clicked.connect(self.addNewTab)
        
        self.setCornerWidget(self.addTabButton)
        
        self.setTabsClosable(True)
        # Connect a tab close signal to close tab slot.
        self.tabCloseRequested[int].connect(self.closeTab)
        
        # Create a single tab initially
        self.addTab(OCMonitorTabView(self), "OAC")
        
    def addNewTab(self):
        newTab = OCMonitorTabView(self)
        self.addTab(newTab, "OAC")
        self.setCurrentWidget(newTab)
    
    def closeTab(self, index):
        if self.count() > 1:
            self.removeTab(index)
        else:
            qApp.quit()


class OCWorkbench(QMainWindow):

    def __init__(self):
        """
        Constructor

        @param parent parent widget
        """
        super(OCWorkbench, self).__init__(None)#, Qt.FramelessWindowHint)

        self.setupWidget()
        #self.createMenus()
        self.setCentralWidget(OCMonitorTabWidget())
        
    def setupWidget(self):
        """Setup the initial properties of this widget"""
        desktop = QApplication.desktop()
        screenRect = desktop.screenGeometry(desktop.primaryScreen())
        windowRect = QRect(0, 0, 800, 600)

        if screenRect.width() < 800:
            windowRect.setWidth(screenRect.width())

        if screenRect.height() < 600:
            windowRect.setHeight(screenRect.height())

        windowRect.moveCenter(screenRect.center())
        self.setGeometry(windowRect)
        self.setMinimumSize(320, 240)
    
        self.setWindowTitle("OpenCog Monitor")
        self.setWindowIcon(QIcon("images/opencog-logo.png"))

    ########################################
    # Create GUI components
    ########################################
    def createMenus(self):
        self.fileMenu = QMenu("&File", self)
        self.actionExit = self.fileMenu.addAction("E&xit")
        self.actionExit.setShortcut(QKeySequence('Ctrl+Q'))

        self.menuBar().addMenu(self.fileMenu)

        self.actionExit.triggered.connect(qApp.quit)


    ##################################################
    # Event handler
    ##################################################
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragPosition = event.globalPos() - self.frameGeometry().topLeft()
            event.accept()

    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self.dragPosition)
            event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    workbench = OCWorkbench()
    workbench.show()
    sys.exit(app.exec_())
