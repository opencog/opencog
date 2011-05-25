#
# Widgets each is in charge of a mind agent within OAC.
# The main job of a widget is getting data from Plaza within OAC and drawing graphs
#
# @author: Zhenhua Cai, czhedu@gmail.com 
# @date:   2011-04-23
#
# @note: I borrowed some code from 
# http://matplotlib.sourceforge.net/examples/user_interfaces/embedding_in_qt4.html
#

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

from PyQt4 import QtGui, QtCore

import zmq
import json

from common import *

import matplotlib as mpl
mpl.rcParams['lines.linewidth'] = 0.5 
mpl.rcParams['font.size'] = 8.0
mpl.rcParams['axes.titlesize'] = 'large'
mpl.rcParams['legend.fancybox'] = True
mpl.rcParams['legend.fontsize'] = 'large'
mpl.rcParams['legend.shadow'] = False
mpl.rcParams['legend.numpoints'] = 1
mpl.rcParams['figure.facecolor'] = 'white'
# for black background
#mpl.rcParams['axes.facecolor'] = 'black'
#mpl.rcParams['axes.edgecolor'] = 'white'
mpl.rcParams['figure.subplot.left']   = 0.05  # the left side of the subplots of the figure
mpl.rcParams['figure.subplot.right']  = 0.95    # the right side of the subplots of the figure
mpl.rcParams['figure.subplot.bottom'] = 0.05    # the bottom of the subplots of the figure
mpl.rcParams['figure.subplot.top']    = 0.90    # the top of the subplots of the figure

class MonitorWidget(FigureCanvas):
    """ Qt4 backend of matplot, which provides a canvas for plotting.
        The actual plotting is done within the MonitorThread class automatically.
    """
    clicked = QtCore.pyqtSignal()

    def __init__(self, publish_endpoint, filter_key, 
                 parent=None, width=5, height=4, dpi=100):

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

        # Initialize variables related to graph 
        self.max_data_len = 50
        # timestamps can be erratic, so we can't assume they'll be
        # evenly spaced
        self.max_time_period = 2000
        self.has_initialized = False

        # expected format for data to be plotted is:
        # { "timestamp": [1,2,...]
        #   "label1" : [val1,val2,...],
        #   "label2" : [val1,val2,...],
        #   }
        self.data_dict = {}
        self.legend_list = []

        # Create and start ZeroMQ subscriber thread
        self.zmq_subscriber_thread = ZmqSubscriberThread(self,
                                                         publish_endpoint, 
                                                         filter_key)
        self.zmq_subscriber_thread.start()

    # Initialize data and legend
    def initialize_data(self, json_dict):
        for k, v in json_dict.iteritems():
            self.data_dict[k] = [v]
            if k != "timestamp":
                self.legend_list.append(k)

        self.has_initialized = True

    # Update data list
    def update_data(self, json_dict):
        latestTimeStamp = json_dict["timestamp"]
        forgetFirst = False
        # forget old data
        if latestTimeStamp - self.data_dict["timestamp"][0] > self.max_time_period or \
            len(self.data_dict["timestamp"]) > self.max_data_len:
            forgetFirst = True
        for k, v in json_dict.iteritems():
            self.data_dict[k].append(v)
            if forgetFirst: self.data_dict[k].pop(0)

    # Draw the graph on the widget
    def draw_graph(self):
        self.axes.clear()
        
        max_t = max(self.data_dict["timestamp"])
        t_minus = [x - max_t for x in self.data_dict["timestamp"]]

        for k in self.data_dict:
            if k == "timestamp": continue
            self.axes.plot(t_minus, self.data_dict[k], '-+')

        leg = self.axes.legend(self.legend_list,
                               'upper left',
                               shadow=True)

        self.axes.set_title(self.zmq_subscriber_thread.filter_key)
        self.axes.grid(True)
        self.axes.set_xlim(-self.max_time_period,0)
        self.axes.set_ylim(0,1)

        self.draw()

    @pyqtSlot(dict)
    def handle_data_update(self, json_dict):
            """
            Process the data in json format
            """
            if not self.has_initialized:
                self.initialize_data(json_dict)

            self.update_data(json_dict)

            # Draw the graph only where no other graph is being rendered.
            # In principle, the global lock is not necessary,
            # however drawing graph is very CPU consuming, 
            # introduce this limit may make GUI more responsive
            if self.isVisible(): 
#                glb.gui_read_write_lock.lockForWrite()
                self.draw_graph()
#                glb.gui_read_write_lock.unlock()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit()

