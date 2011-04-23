# -*- coding: utf-8 -*-

# An elegant chrome-like openpsi monitor browser.
#
# @author Troy Huang <huangdeheng@gmail.com>
# @date 2011-03-30
# @update 2011-04-19



import sys
import zmq

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from common import *

from monitor_widget import *

class OCMonitorPanel(QFrame):
    """
    OCMonitorPanel is the container of MonitorWidgets.
    View of MonitorWidgets can be changed through this panel.
    """
    def __init__(self, parent=None, columnNum=2):
        super(OCMonitorPanel, self).__init__(parent)

        self.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding, True)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)
        
        self.monitorList = []
        self.columnNum = columnNum

        self.soloist = None
        self.zmqUrl = None

        gridLayout = QGridLayout(self)

        self.setLayout(gridLayout)

    def addMonitors(self, monitors):
        oldCount = len(self.monitorList)
        self.monitorList.extend(monitors)
        for i, monitor in enumerate(monitors):
            row = (oldCount + i) / self.columnNum
            col = (oldCount + i) % self.columnNum
            self.layout().addWidget(monitor, row, col)
            monitor.clicked.connect(self.showSoloView)

    def createMonitors(self, zmqUrl=""):
        if zmqUrl == "" or (not str(zmqUrl).startswith("tcp://") and\
            not str(zmqUrl).startswith("ipc://")):
            warning = QMessageBox.warning(self, "Warning",
                    "Please input a valid zmq url starting with "
                    "tcp:// or ipc://")
            return

        if self.soloist != None:
            self.showGroupView()

        if not self.zmqUrl or self.zmqUrl != zmqUrl:
            self.zmqUrl = zmqUrl
            if len(self.monitorList) > 0:
                for monitor in self.monitorList:
                    monitor.hide()
                    #self.layout().removeWidget(monitor)
                del self.monitorList[:]
        else:
            return 
        
        monitor1 = MonitorWidget(str(self.zmqUrl), "PsiModulatorUpdaterAgent")
        monitor2 = MonitorWidget(str(self.zmqUrl), "PsiFeelingUpdaterAgent")
        monitor3 = MonitorWidget(str(self.zmqUrl), "PsiDemandUpdaterAgent")
                                
        self.addMonitors([monitor1, \
                          monitor2, \
                          monitor3]) 

    def showSoloView(self):
        sender = self.sender()

        if self.soloist != sender:
            self.soloist = sender
            self.soloistOldSize = QSize(sender.size())
        else:
            return

        for monitor in self.monitorList:
            if sender != monitor:
                monitor.hide()
        sender.update()
        sender.updateGeometry()


    def updateSoloView(self):
        """
        Update solo view when there's a resize event
        """
        rect = QRect(0, 0, self.geometry().width(), self.geometry().height())
        self.soloist.setGeometry(rect)

    def showGroupView(self):
        self.soloist = None
        for monitor in self.monitorList:
            monitor.show()

class OCMonitorTabView(QWidget):
    """
    The tab view include two components: navigation bar and monitor panel.
    """
    def __init__(self, parent=None):
        super(OCMonitorTabView, self).__init__(parent)

        # Initialize a navigation bar
        self.navigationBar = QToolBar(self)
        
        # Initialize a monitor panel
        self.monitorPanel = OCMonitorPanel(self)
        
        self.createNavigationBar()

        tabLayout = QVBoxLayout(self)
        tabLayout.addWidget(self.navigationBar, 0, Qt.AlignTop)
        tabLayout.addWidget(self.monitorPanel, 1)
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
    

class OCMonitorBrowser(QTabWidget):
    def __init__(self, parent=None):
        super(OCMonitorBrowser, self).__init__(parent)
        
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
        self.addTab(OCMonitorTabView(), "OAC")
        
    def addNewTab(self):
        newTab = OCMonitorTabView()
        self.addTab(newTab, "OAC")
        self.setCurrentWidget(newTab)
    
    def closeTab(self, index):
        if self.count() > 1:
            self.removeTab(index)
        else:
            qApp.quit()

class OCMonitorBrowserStandalone(QMainWindow):
    """This class is used to test monitor browser without the 
    wrapper of OCWorkbench.
    """
    def __init__(self):
        """
        Constructor

        @param parent parent widget
        """
        super(OCMonitorBrowserStandalone, self).__init__(None)#, Qt.FramelessWindowHint)

        self.setupWidget()
        #self.createMenus()
        self.centralWidget = QWidget(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHeightForWidth(self.centralWidget.sizePolicy().hasHeightForWidth())
        self.centralWidget.setSizePolicy(sizePolicy)
        self.mainLayout = QGridLayout(self.centralWidget)

        monitorTabWidget = OCMonitorBrowser(self.centralWidget)

        self.mainLayout.addWidget(monitorTabWidget, 0, 0)
        self.setCentralWidget(self.centralWidget)
        
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
    
        self.setWindowTitle("OpenCog Monitor Browser")
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
    standaloneBrowser = OCMonitorBrowserStandalone()
    standaloneBrowser.show()
    sys.exit(app.exec_())
