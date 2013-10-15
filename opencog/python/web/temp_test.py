__author__ = 'Cosmo Harrigan'

#import daemon
from opencog.atomspace import *
from web.api import RESTApi
import sys, time
from daemon2 import Daemon

'''
atomspace = AtomSpace()
animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))

api = RESTApi(atomspace)
'''
#client = api.run()

#daemon.spawn_daemon(api.run)


class MyDaemon(Daemon):
    def run(self):
        print 'entering run'
        atomspace = AtomSpace()
        animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
        api = RESTApi(atomspace)
        print 'going to start api'
        api.run()
        print 'api started'
        while True:
            time.sleep(1)

if __name__ == "__main__":
    daemon = MyDaemon('/tmp/daemon-example.pid')
    if len(sys.argv) == 2:
        if 'start' == sys.argv[1]:
            daemon.start()
        elif 'stop' == sys.argv[1]:
            daemon.stop()
        elif 'restart' == sys.argv[1]:
            daemon.restart()
        else:
            print "Unknown command"
            sys.exit(2)

        sys.exit(0)
    else:
        print "usage: %s start|stop|restart" % sys.argv[0]

    sys.exit(2)

    print 'done'

    sleep(60)

    print 'exiting'