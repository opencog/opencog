__author__ = 'Cosmo Harrigan'

import daemon
from opencog.atomspace import *
from web.api import RESTApi
from time import sleep

#def sayhi(atomspace_param):


atomspace = AtomSpace()
animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))

api = RESTApi(atomspace)
#client = api.run()

daemon.spawn_daemon(api.run)
print 'done'

sleep(60)

print 'exiting'