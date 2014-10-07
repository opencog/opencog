__author__ = 'Cosmo Harrigan'

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

    Prerequisites:
        1) Requires installation of the Python dependencies by running:
            sudo ./install_dependencies.sh

        2) Requires the configuration file (opencog.conf) to contain the
           following parameters:
            - PYTHON_EXTENSION_DIRS must specify the relative location of the
              API scripts
                Example: PYTHON_EXTENSION_DIRS = ../opencog/python/web/api
            - PYTHON_PRELOAD must specify the restapi module
                Example: PYTHON_PRELOAD = restapi

    To start the REST API, type restapi.Start at the CogServer shell
    """

    summary = "Start the OpenCog REST API"
    description = "Usage: restapi.Start\n\nStarts the OpenCog REST API. " \
                  "This will provide a REST interface to the Atomspace,\n" \
                  "allowing you to create, read, update and delete atoms " \
                  "across the network using\nHTTP requests/responses with " \
                  "JSON-formatted data.\n\nDefault endpoint: " \
                  "http://127.0.0.1:5000/api/v1.1/\nExample request: " \
                  "http://127.0.0.1:5000/api/v1.1/atoms?type=ConceptNode"

    def run(self, args, atomspace):
        """
        Loads the REST API into a separate thread and invokes it, so that it
        will continue serving requests in the background after the Request
        that loads it has returned control to the CogServer
        """
        self.atomspace = atomspace
        thread = Thread(target=self.invoke)
        thread.start()
        print "REST API is now running in a separate thread."
        # @todo: detect Control-C to end the thread

    def invoke(self):
        self.api = RESTAPI(self.atomspace)
        self.api.run(host=IP_ADDRESS, port=PORT)
