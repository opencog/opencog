"""
AtomSpace Publisher Extension for socket.io
Extends: http://wiki.opencog.org/wikihome/index.php?title=AtomSpace_Event_Publisher

Forwards ZeroMQ messages from the AtomSpace Publisher module
to a socket.io socket accessible from web applications using
JavaScript and JQuery, including cross-domain requests

Dependencies:
gevent:
sudo apt-get install python-gevent

gevent-socketio:
pip install gevent-socketio

Instructions:

1) Important note:
   For an unknown reason, tcp6 steals port 17001
   Because of that, you must choose a different value for SERVER_PORT in opencog.conf

2) Start the CogServer

3) Ensure that libatomspacepublishermodule.so is loaded: you will need to include it
in the list of active modules in the MODULES section of opencog.conf. The module
path is: opencog/persist/zmq/events/libatomspacepublishermodule.so

4) Browse to: http://localhost:8080/ to load an example client, defined in 'index.html'

5) Perform atomspace operations, and you should see updates in the browser

For more details on how to work with socket.io, visit:
http://socket.io/
"""

__author__ = 'Cosmo Harrigan'

# ZeroMQ
ZMQ_IP_ADDRESS = '127.0.0.1'
ZMQ_PORT = '5563'

import zmq.green as zmq
from gevent import monkey
monkey.patch_all()

from socketio import socketio_manage
from socketio.server import SocketIOServer
from socketio.namespace import BaseNamespace
from socketio.mixins import BroadcastMixin

# ZeroMQ subscribers setup
context = zmq.Context(1)
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://" + ZMQ_IP_ADDRESS + ':' + ZMQ_PORT)
subscriber.setsockopt(zmq.SUBSCRIBE, "add")
subscriber.setsockopt(zmq.SUBSCRIBE, "remove")
subscriber.setsockopt(zmq.SUBSCRIBE, "tvchanged")
subscriber.setsockopt(zmq.SUBSCRIBE, "avchanged")


class AtomSpaceNamespace(BaseNamespace, BroadcastMixin):
    def recv_connect(self):
        print 'ZeroMQ listener initialized'
        while True:
            [address, contents] = subscriber.recv_multipart()
            print("[%s] %s" % (address, contents))
            self.emit(address, contents)


class Application(object):
    def __init__(self):
        self.buffer = []

    def __call__(self, environ, start_response):
        path = environ['PATH_INFO'].strip('/') or 'index.html'

        if path.startswith('static/') or path == 'index.html':
            try:
                data = open(path).read()
            except Exception:
                return not_found(start_response)

            if path.endswith(".js"):
                content_type = "text/javascript"
            elif path.endswith(".css"):
                content_type = "text/css"
            elif path.endswith(".swf"):
                content_type = "application/x-shockwave-flash"
            else:
                content_type = "text/html"

            start_response('200 OK', [('Content-Type', content_type)])
            return [data]

        if path.startswith("socket.io"):
            socketio_manage(environ, {'/atomspace': AtomSpaceNamespace})
        else:
            return not_found(start_response)


def not_found(start_response):
    start_response('404 Not Found', [])
    return ['<h1>Not Found</h1>']

if __name__ == '__main__':
    print 'Listening on port http://0.0.0.0:8080 and on port 10843 (flash policy server)'
    SocketIOServer(('0.0.0.0', 8080), Application(),
                   resource="socket.io", policy_server=True,
                   policy_listener=('0.0.0.0', 10843)).serve_forever()
