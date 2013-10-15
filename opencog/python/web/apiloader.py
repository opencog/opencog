__author__ = 'Cosmo Harrigan'

import opencog.cogserver
from web.api import RESTApi
from threading import Thread

class RESTApiThread(object):
    @classmethod
    def configure(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def run(self):
        api = RESTApi(self.atomspace)
        print 'Starting API...'
        api.run()

class RESTApiLoader(opencog.cogserver.Request): #opencog.cogserver.MindAgent):
    def run(self, args, atomspace):
        RESTApiThread.configure(atomspace)
        #daemon = ApiDaemon('/tmp/daemon-opencog-rest.pid')
        #daemon.start()
        api = RESTApiThread()
        thread = Thread(target=api.run)
        thread.start()
        print "REST API is now running in a separate thread."

'''
    def invoke(self, atomspace):
        api = RESTApi(atomspace)
        api.run()
'''







'''
aspace = AtomSpace()
animal = aspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
a = RESTApiLoader()
a.run(args=None, atomspace=aspace)
'''