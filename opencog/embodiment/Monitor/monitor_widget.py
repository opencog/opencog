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

        # Initialize variables related to graph 
        self.max_data_len = 50
        self.has_initialized = False
        self.data_dict = {}
        self.legend_list = []
    
    # Initialize data and legend
    def initialize_data(self, json_dict):
        for k, v in json_dict.iteritems():
            self.data_dict[k] = [v]
            if k!="timestamp":
                self.legend_list.append(k)

        self.has_initialized = True

    # Update data list
    def update_data(self, json_dict):
        for k, v in json_dict.iteritems():
            self.data_dict[k].append(v)
            if (len(self.data_dict[k]) > self.max_data_len):
                self.data_dict[k].pop(0)

    # Draw the graph on the widget
    def draw_graph(self):
        MonitorThread.read_write_lock.lockForWrite()

        self.widget.axes.clear()

        for k in self.data_dict.keys():
            if (k!="timestamp"):
                self.widget.axes.plot(self.data_dict["timestamp"], self.data_dict[k])

        leg = self.widget.axes.legend(self.legend_list,
                                      'upper left',
                                      shadow=True
                                     )

        self.widget.draw()

        MonitorThread.read_write_lock.unlock()

    # Execution entrance of the thread
    def run(self):
        while True:
            message = self.socket.recv()

            # if the message contains only filter key, discard it
            if message == self.filter_key: continue
           
            json_dict = json.loads(message) 

            if not self.has_initialized:
                self.initialize_data(json_dict)
                continue

            self.update_data(json_dict)
            self.draw_graph()

#            self.usleep(50)

# A lock used to synchronize all the monitor threads, 
# We should avoid drawing on the widgets at the same time 
MonitorThread.read_write_lock = QtCore.QReadWriteLock()

