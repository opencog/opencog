import opencog.cogserver

class TestAgent(opencog.cogserver.MindAgent):

    def run(self,a):
        print "I am running and the atomspace contains:" + a

