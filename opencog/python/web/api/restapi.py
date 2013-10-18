__author__ = 'Cosmo Harrigan'

import opencog.cogserver
from web.api.apimain import RESTAPI
from threading import Thread


class Start(opencog.cogserver.Request):
    """
    Implements a CogServer Module to load upon startup that will load the REST API defined in apimain.py.
    Requires the configuration file (opencog.conf) to contain the following parameters:
        1) PYTHON_EXTENSION_DIRS must specify the relative location of the API scripts
          Example: PYTHON_EXTENSION_DIRS = ../opencog/python/web/rest
        2) PYTHON_PRELOAD must specify the restapi module
          Example: PYTHON_PRELOAD = restapi

    To start the REST API, type restapi.Start at the CogServer shell
    """

    summary = "Start the OpenCog REST API"
    description = "Usage: restapi.Start\n\n" \
    "Starts the OpenCog REST API. This will provide a REST interface to the Atomspace,\n" \
    "allowing you to create, read, update and delete atoms across the network using\n" \
    "HTTP requests/responses with JSON-formatted data.\n\n" \
    "Default endpoint: http://127.0.0.1:5000/api/v1.0/\n" \
    "Example request: http://127.0.0.1:5000/api/v1.0/atoms?type=ConceptNode"

    def run(self, args, atomspace):
        """
        Loads the REST API into a separate thread and invokes it, so that it will continue serving requests in the
        background after the Request that loads it has returned control to the CogServer
        """
        self.atomspace = atomspace
        thread = Thread(target=self.invoke)
        thread.start()
        print "REST API is now running in a separate thread."

    def invoke(self):
        api = RESTAPI(self.atomspace)
        api.run()

