#
# Widgets each is in charge of a mind agent within OAC.
# The main job of a widget is getting data from Plaza within OAC and drawing graphs
#
# @author: Zhenhua Cai, czhedu@gmail.com 
# @date:   2011-03-30
#
# @note: I borrowed some code from 
# http://matplotlib.sourceforge.net/examples/user_interfaces/embedding_in_qt4.html
#

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

from PyQt4 import QtGui, QtCore

import zmq
import json

from collections import deque

class MonitorWidget(FigureCanvas):
    """ Qt4 backend of matplot, which provides a canvas for plotting.
        The actual plotting is done within the MonitorThread class automatically.
    """
    def __init__(self, parent=None, width=5, height=4, dpi=100):

        # Initialize figure canvas
        self.figure = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.figure.add_subplot(111)
        self.axes.hold(True) # Axes would be cleared each time plot() is called

        FigureCanvas.__init__(self, self.figure)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, 
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding
                                  )
        FigureCanvas.updateGeometry(self)

class MonitorThread(QtCore.QThread):
    """ Get data from Plaza within OAC via ZeroMQ and draw graphs on MonitorWidget
    """
    def __init__(self, zmq_context, publish_endpoint, filter_key,
                 parent=None, width=5, height=4, dpi=100):

        # Initialize the thread and locker
        QtCore.QThread.__init__(self)

        # Create a figure canvas
        self.widget = MonitorWidget(parent, width, height, dpi)

        # Initialize the zeromq subscriber (socket)
        self.socket = zmq_context.socket(zmq.SUB)
        self.socket.connect(publish_endpoint)
        self.filter_key = filter_key
#        self.signal_name = "signal_" + filter_key
        self.socket.setsockopt(zmq.SUBSCRIBE, self.filter_key)

        self.max_data_len = 50
        self.x_time = []
        self.y_activation = []
        self.y_resolution = []
        self.y_securing_threshold = []
        self.y_selection_threshold = []

    def run(self):
        while True:
            message = self.socket.recv()
            if message == self.filter_key: continue

#            print message

            data = json.loads(message) 

            self.x_time.append(data['timestamp'])
            self.y_activation.append(data['Activation'])
            self.y_resolution.append(data['Resolution'])
            self.y_securing_threshold.append(data['SecuringThreshold'])
            self.y_selection_threshold.append(data['SelectionThreshold'])

            if len(self.x_time) > self.max_data_len:
                self.x_time.pop(0)
                self.y_activation.pop(0)
                self.y_resolution.pop(0)
                self.y_securing_threshold.pop(0)
                self.y_selection_threshold.pop(0)

            MonitorThread.read_write_lock.lockForWrite()

            self.widget.axes.clear()

            self.widget.axes.plot(self.x_time, self.y_activation)
            self.widget.axes.plot(self.x_time, self.y_resolution)
            self.widget.axes.plot(self.x_time, self.y_securing_threshold)
            self.widget.axes.plot(self.x_time, self.y_selection_threshold)

            leg = self.widget.axes.legend(('Activation', 
                                           'Resolution',
                                           'SecuringThreshold', 
                                           'SelectionThreshold'
                                           ),
                                           'upper left',
                                           shadow=True
                                         )

            self.widget.draw()

            MonitorThread.read_write_lock.unlock()

            self.usleep(50)

MonitorThread.read_write_lock = QtCore.QReadWriteLock()

