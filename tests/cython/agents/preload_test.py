import opencog.cogserver
from opencog.atomspace import types

print "Preloaded " + __name__

class PreloadTestAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        pass

    def run(self,atomspace):
        print "running agent from preloaded file"
