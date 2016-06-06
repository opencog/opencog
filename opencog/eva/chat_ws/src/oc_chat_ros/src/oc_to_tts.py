#!/usr/bin/env python

#server listens to string from opencog and sends a ros message to tts
import SocketServer
import rospy
from std_msgs.msg import String

global pub 

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        global pub
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print "{} wrote:".format(self.client_address[0])
        print self.data
        # just send back the same data, but upper-cased
        #self.request.sendall(self.data.upper())
        #pub = rospy.Publisher('/tts', String, queue_size=5)
        pub.publish(self.data)

if __name__ == "__main__":
    global pub
    HOST, PORT = "localhost", 17030
    rospy.init_node('oc_chat_tts',anonymous=True)
    pub = rospy.Publisher('/tts', String, queue_size=5)
    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever(poll_interval=0.5)

