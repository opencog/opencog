#
# Common classes, variables and functions shared by other source code
#
# @author Zhenhua Cai <czhedu@gmail.com>
# @date   2011-04-23
#

import sys
import zmq
import json

from PyQt4.QtCore import *

###############################################################################

class glb:
    """
    Global variables and functions
    Add your global variables in function 'init_global_var'
    Add your global functions with @staticmethod decoration
    Don't create any instance of class 'glb', just use the class directly!
    Such as glb.init_global_var(), glb.zmq_context etc.
    """

    # ZeroMQ i/o context shared by all the ZeroMQ sockets 
    zmq_context = zmq.Context()

    def __init__(self):
        pass

    @staticmethod
    def init_global_var():
        """
        Initialize all the global variables.
        This shall be called only once at the beginning of the whole application
        """

        # A lock used to synchronize all the monitor threads, 
        # We should avoid drawing on the widgets at the same time
        glb.gui_read_write_lock = QReadWriteLock()

###############################################################################

class ZmqSubscriberThread(QThread):

    data_update_signal = pyqtSignal(dict)

    def __init__(self, widget, publish_endpoint, filter_key,
                 zmq_context = glb.zmq_context, parent = None):

        """
        widget should contains the slot method as below:
            @pyqtSlot(dict)
            def handle_data_update(self, json_dict):
                # Some code here to process the data in json format

        publish_endpoint tells the subscriber where the message source is

        filter_key would make the subscriber only get the message with matching filter_key 
        """

        # Initialize the thread
        QThread.__init__(self)

        # Connect the signal with the handler residing in widget 
        self.widget = widget
        self.data_update_signal.connect(self.widget.handle_data_update)

        # Initialize the ZeroMQ socket 
        self.socket = zmq_context.socket(zmq.SUB)
        self.socket.connect(publish_endpoint)
        self.filter_key = filter_key
        self.socket.setsockopt(zmq.SUBSCRIBE, self.filter_key)

    def run(self):
        """
        Receive the message with matching filter_key from publish_endpoint 
        via ZeroMQ, discard the filter_key message and emit the signal to 
        corresponding handler with the actual data wrapped in python dictionary
        """
        while True:
            message = self.socket.recv()
         
            # if the message contains only filter key, discard it
            if message == self.filter_key: continue
          
            # Unpack the message into python dictionary
            json_dict = json.loads(message)
           
            # Emit the signal which would evoke the corresponding handler
            self.data_update_signal.emit(json_dict)

