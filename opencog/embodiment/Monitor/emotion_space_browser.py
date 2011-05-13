import numpy as np
import zmq
import json

import pylab
import matplotlib as mpl
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

# configure the matplotlib settings
#mpl.rcParams['legend.fontsize'] = 10

from PyQt4 import QtGui, QtCore

from common import *

class ZmqMultiFiltersSubscriberThread(QThread):

    data_update_signal = pyqtSignal(dict)

    def __init__(self, widget, publish_endpoint, filter_list,
                 zmq_context = glb.zmq_context, parent = None):

        """
        widget should contains the slot method as below:
            @pyqtSlot(dict)
            def handle_data_update(self, json_dict):
                # Some code here to process the data in json format

        publish_endpoint tells the subscriber where the message source is

        """

        # Initialize the thread
        QThread.__init__(self)

        # Connect the signal with the handler residing in widget 
        self.widget = widget
        self.data_update_signal.connect(self.widget.handle_data_update)

        # Initialize the ZeroMQ socket 
        self.socket = zmq_context.socket(zmq.SUB)

        self.filter_list = filter_list
        for filter_name in self.filter_list:
            self.socket.setsockopt(zmq.SUBSCRIBE, filter_name)

        self.socket.connect(publish_endpoint)

    def run(self):
        """
        Receive the message with matching filter_key from publish_endpoint 
        via ZeroMQ, discard the filter_key message and emit the signal to 
        corresponding handler with the actual data wrapped in python dictionary
        """
        while True:
            message = self.socket.recv()
         
            # if the message contains only filter key, discard it
            if message in self.filter_list:
                self.latest_msg_filter = message
                continue
          
            # Unpack the message into python dictionary
            json_dict = json.loads(message)

            # Apply a filter name to this data dictionary, in order to distinguish it
            json_dict['filter_key'] = self.latest_msg_filter
           
            # Emit the signal which would evoke the corresponding handler
            self.data_update_signal.emit(json_dict)

class EmotionSpace(FigureCanvas):
    def __init__(self, publish_endpoint, parent=None, width=5, height=4, dpi=100):

        # Initialize a cache for incoming data.
        self.max_data_len = 25
        
        ## Feeling dictionary stores dominant feelings with different timestamps.
        ## Format: { timestamp -> dominant_feeling_name }
        self.feeling_dict = {}

        ## Modulator dictionary caches modulators value at different time points.
        ## Format:
        ##      { modulator_name -> { timestamp -> modulator_value } }
        self.modulator_dict = {}

        # The modulator dicitonary should be initialized in the format before 
        # appending data.
        self.has_modulator_dict_initialized = False

        # The legend list used to show legend in the chart
        self.legend_list = []

        # Chosen 3 modulators to be the axes of 3-dimensional space.
        self.modulator_axes = ['Activation', 'Resolution', 'SecuringThreshold']
        # Initialize variables related to graphics.
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = Axes3D(self.fig)
        self.axes.hold(False)
        
        FigureCanvas.__init__(self, self.fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, 
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding
                                  )
        FigureCanvas.updateGeometry(self)

        # Create and start ZeroMQ subscriber threads
        self.zmq_sub_thread = ZmqMultiFiltersSubscriberThread(self,
                                                            publish_endpoint, 
                                                            [
                                                              "PsiFeelingUpdaterAgent",
                                                              "PsiModulatorUpdaterAgent"
                                                            ]
                                                             )
        self.zmq_sub_thread.start()

    # Initialize modulator dictionary and legend list
    def initialize_modulator_dict(self, json_dict):
        timestamp = json_dict['timestamp']
        del json_dict['timestamp']
        for k, v in json_dict.iteritems():
            self.modulator_dict[k] = {}
            self.modulator_dict[k][timestamp] = v
            
            self.legend_list.append(k)

        self.has_modulator_dict_initialized = True

    def update_data(self, json_dict):
        if json_dict['filter_key'] == "PsiFeelingUpdaterAgent":
            # Just leave feelings alone
            del json_dict['filter_key']
            timestamp = json_dict['timestamp']
            del json_dict['timestamp']

            # Get the feeling name with max value
            dominant_feeling = max(json_dict, key = lambda k : json_dict.get(k))

            # Cache the pair in the feeling dictionary
            self.feeling_dict[timestamp] = dominant_feeling
            return 0


        elif json_dict['filter_key'] == "PsiModulatorUpdaterAgent":
            # Remove filter key pair
            del json_dict['filter_key']

            if not self.has_modulator_dict_initialized:
                self.initialize_modulator_dict(json_dict)
                return

            timestamp = json_dict['timestamp']
            del json_dict['timestamp']
            for k, v in json_dict.iteritems():
                self.modulator_dict[k][timestamp] = v
            return 1
        else:
            pass
        

    @pyqtSlot(dict)
    def handle_data_update(self, json_dict):
        """
        Process the data in json format
        """
        
        update_seq = self.update_data(json_dict)

        if self.isVisible() and update_seq == 1:
            self.do_draw()

    def do_draw(self):
        self.axes.clear()

        X = []
        Y = []
        Z = []

        m = self.modulator_axes
        print '=========='
        for k, v in self.feeling_dict.iteritems():
            X.append(self.modulator_dict[m[0]][k])
            Y.append(self.modulator_dict[m[1]][k])
            Z.append(self.modulator_dict[m[2]][k])

            print str(self.modulator_dict[m[0]][k]) + ':' \
                    + str(self.modulator_dict[m[1]][k]) + ':' \
                    + str(self.modulator_dict[m[2]][k])
        print '=========='

        self.axes.grid(True)
        self.axes.plot(X, Y, Z, '-o')
        self.draw()
