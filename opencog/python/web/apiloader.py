__author__ = 'Cosmo Harrigan'

import opencog.cogserver
from web.api import RESTApi
from threading import Thread


class RESTApiLoader(opencog.cogserver.Request): #opencog.cogserver.MindAgent):
    def run(self, args, atomspace):
        """
        Load the REST Api into a separate thread so that it will continue serving requests in the background
        after the Request that loads it has terminated
        """
        self.atomspace = atomspace
        thread = Thread(target=self.invoke)
        thread.start()
        print "REST API is now running in a separate thread."

    def invoke(self):
        api = RESTApi(self.atomspace)
        api.run()

