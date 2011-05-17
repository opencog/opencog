# -*- coding: utf-8 -*-
#!/usr/bin/env python

# An elegant workbench GUI for OpenCog.
#
# @author Troy Huang <huangdeheng@gmail.com>
# @date 2011-04-18
#



import sys

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from common import *

from monitor_browser import *
from nlp_browser import *
from emotion_space_browser import *

# Initialize global variables
glb.init_global_var()

class OCWorkbench(QMainWindow):
    """The main window of OpenCog workbench.
    """
    def __init__(self):
        """Constructor
        """
        super(OCWorkbench, self).__init__(None)#, Qt.FramelessWindowHint)

        self.setupWorkbench()
        #self.createMenus()
        
        # Create a central widget to contain module switchs and scenes.
        self.centralWidget = QWidget(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralWidget.sizePolicy().hasHeightForWidth())
        self.centralWidget.setSizePolicy(sizePolicy)

        # Layout for central widget.
        self.centralLayout = QHBoxLayout(self.centralWidget)

        # A list of module icons to switch scenes
        self.moduleIconList = QListWidget()
        self.moduleIconList.setViewMode(QListView.IconMode)
        self.moduleIconList.setIconSize(QSize(96, 96))
        self.moduleIconList.setMovement(QListView.Static)
        self.moduleIconList.setMaximumWidth(128)
        self.moduleIconList.setMinimumWidth(128)
        self.moduleIconList.setSpacing(12)

        self.createModuleIconList()
        # A stack of scenes that can be switched.
        self.sceneStack = QStackedWidget()
        # Psi Monitor Widget
        self.sceneStack.addWidget(OCMonitorBrowser())
        # Dialog System Widget
        self.sceneStack.addWidget(NLPBrowser())
        # Emotion Space Widget
        emotion_space = EmotionSpace("tcp://127.0.0.1:18002")
        self.sceneStack.addWidget(emotion_space)
        
        self.centralLayout.addWidget(self.moduleIconList)
        self.centralLayout.addWidget(self.sceneStack)

        #self.monitorTabWidget = OCMonitorTabWidget(self.frame)
        self.setCentralWidget(self.centralWidget)

        # Create the menu
        self.createMenus()
        
    def setupWorkbench(self):
        """Setup the initial properties of workbench"""
        desktop = QApplication.desktop()
        screenRect = desktop.screenGeometry(desktop.primaryScreen())
        windowRect = QRect(0, 0, 1024, 768)

        if screenRect.width() < 1024:
            windowRect.setWidth(screenRect.width())

        if screenRect.height() < 768:
            windowRect.setHeight(screenRect.height())

        windowRect.moveCenter(screenRect.center())
        self.setGeometry(windowRect)
        self.setMinimumSize(800, 600)
    
        self.setWindowTitle("OpenCog Workbench")
        self.setWindowIcon(QIcon('images/opencog-logo.png'))

    def changeScene(self, current, previous):
        """A slot used to change scene when clicking on the module icon."""
        if not current:
            current = previous

        self.sceneStack.setCurrentIndex(self.moduleIconList.row(current))

    def createModuleIconList(self):
        # icon for psi monitor
        monitorButton = QListWidgetItem(self.moduleIconList)
        monitorButton.setIcon(QIcon('images/psi-monitor.png'))
        monitorButton.setText("Psi Monitor")
        monitorButton.setTextAlignment(Qt.AlignHCenter)
        monitorButton.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        # icon for dialog system
        dialogButton = QListWidgetItem(self.moduleIconList)
        dialogButton.setIcon(QIcon('images/language1.png'))
        dialogButton.setText("Dialog System")
        dialogButton.setTextAlignment(Qt.AlignHCenter)
        dialogButton.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        #icon for emotion space browser
        emotionButton = QListWidgetItem(self.moduleIconList)
        emotionButton.setIcon(QIcon('images/heart-beat.png'))
        emotionButton.setText("Emotion Space")
        emotionButton.setTextAlignment(Qt.AlignHCenter)
        emotionButton.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)

        self.moduleIconList.currentItemChanged.connect(self.changeScene)

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

    # Obsolete method.
    def createToolBox(self):
        self.toolBox = QTabWidget(self.centralWidget)
        # define size policy of tool box.
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolBox.sizePolicy().hasHeightForWidth())
        self.toolBox.setSizePolicy(sizePolicy)
        self.toolBox.setMinimumSize(QSize(512, 512))
        self.toolBox.setTabPosition(QTabWidget.West)
        self.toolBox.setTabShape(QTabWidget.Rounded)

        self.toolBox.addTab(QLabel("RelEx"), "Language")
        self.toolBox.addTab(OCMonitorTabWidget(), "OAC")

        self.toolBox.setTabIcon(1, QIcon("images/psi_monitor.png"))

        self.toolBox.setStyleSheet("QTabBar::tab {min-width: 128px; \
                        text-align: center; \
                        border: 1px solid #333; \
                        margin-top: 4px; \
                        padding:6px;}")

    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    workbench = OCWorkbench()
    workbench.show()
    sys.exit(app.exec_())
