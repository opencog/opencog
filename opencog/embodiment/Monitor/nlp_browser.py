#
# Monitor of Natural Language Processing (RelexServer and NLGen)
#
# @author Zhenhua Cai <czhedu@gmail.com>
#
# @date 2011-04-23
#

import sys
import time
import zmq
import json

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from common import *

###############################################################################

class RelexServerMonitorWidget(QTextEdit):
    def __init__(self, parent=None):

        # Initialize the text editor
        super(RelexServerMonitorWidget, self).__init__(parent)
        self.setHtml("RelexServer Monitor <br> Please run <br> python ./test_server.py")
        self.setReadOnly(True)

        # Create and start ZeroMQ subscriber thread
        self.zmq_subscriber_thread = ZmqSubscriberThread(self, 
                                                         "tcp://127.0.0.1:16316", 
                                                         "RelexServer"
                                                        )
        self.zmq_subscriber_thread.start()

    @pyqtSlot(dict)
    def handle_data_update(self, json_dict):
        """
        Display the data in json format
        """
        html_text = '<font color="#002EB8"> ' + \
                     '<strong> Sender Agent Id: </strong> ' + \
                      str(json_dict['sender_agent_id']) + \
                     '</font> <br> <br>'

        html_text += '<font color="#66FF33"> ' + \
                     '<strong> Receiver Agent Id: </strong> ' + \
                      str(json_dict['receiver_agent_id']) + \
                     '</font> <br> <br>'

        html_text += '<font color="#6633FF"> ' + \
                     '<strong> Content Type: </strong> ' + \
                      json_dict['content_type'] + \
                     '</font> <br> <br>'

        html_text += '<font color="#F5B800"> ' + \
                     '<strong> Target Mode: </strong> ' + \
                      json_dict['target_mode'] + \
                     '</font> <br> <br>'

        html_text += '<font color="#000000"> ' + \
                     '<strong> Original Sentence: </strong> <br>' + \
                      json_dict['original_sentence'] + \
                     '</font> <br> <br>'

        self.setHtml(html_text)

###############################################################################

# TODO: Finish it later
class NLGenMonitorWidget(QTextEdit):
    def __init__(self, parent=None):

        # Initialize the text editor
        super(NLGenMonitorWidget, self).__init__(parent)
        self.setText("NLGenServer Monitor")
        self.setReadOnly(True)

        # Create and start ZeroMQ subscriber thread
        self.zmq_subscriber_thread = ZmqSubscriberThread(self, 
                                                         "tcp://127.0.0.1:16317", 
                                                         "NLGenServer"
                                                        )
        self.zmq_subscriber_thread.start()

    @pyqtSlot(dict)
    def handle_data_update(self, json_dict):
        """
        Display the data in json format
        """

###############################################################################

class NLPBrowser(QWidget):
    """
    Natural Language Processing monitor
    """
    def __init__(self, parent=None):
        super(NLPBrowser, self).__init__(parent)

        h_box_layout = QHBoxLayout(self)
        h_box_layout.addWidget(RelexServerMonitorWidget())
        h_box_layout.addWidget(NLGenMonitorWidget())
        self.setLayout(h_box_layout)

