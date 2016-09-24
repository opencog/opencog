__author__ = 'Cosmo Harrigan'

import socket
import opencog.cogserver
from web.api.apimain import RESTAPI
from threading import Thread

# Endpoint configuration
# To allow public access, set to 0.0.0.0; for local access, set to 127.0.0.1
IP_ADDRESS = '0.0.0.0'
PORT = 5000


class Start(opencog.cogserver.Request):
    """
    Implements a CogServer Module to load upon startup that will load the REST
    API defined in apimain.py

    See examples @ github.com/opencog/opencog/tree/master/examples/restapi
    """

    summary = "Start the OpenCog REST API"
    description = "Usage: restapi.Start\n\nStarts the OpenCog REST API. " \
                  "This will provide a REST interface to the Atomspace,\n" \
                  "allowing you to create, read, update and delete atoms " \
                  "across the network using\nHTTP requests/responses with " \
                  "JSON-formatted data.\n\nDefault endpoint: " \
                  "http://127.0.0.1:5000/api/v1.1/\nExample request: " \
                  "http://127.0.0.1:5000/api/v1.1/atoms?type=ConceptNode"

    def __init__(self):
        self.atomspace = None  # Will be passed as argument in run method

    def run(self, args, atomspace):
        self.atomspace = atomspace
        '''
        make a daemon thread so that it can be interrupted
        '''
        thread = Thread(target=self.invoke)
        thread.setDaemon(True)
        thread.start()
        print "REST API is now running in a separate daemon thread."

    def invoke(self):
        self.api = RESTAPI(self.atomspace)

        # OK, so if the remote end closes the pipe, we get a SIGPIPE
        # error, and the server dies.  So just restart the server in
        # that situation. See bug opencog/ros-behavior-scripting/issues/108
        try_again = True
        while try_again:
            try_again = False
            try:
                self.api.run(host=IP_ADDRESS, port=PORT)
            except socket.error, e:
                try_again = True
