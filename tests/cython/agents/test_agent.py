import opencog.cogserver
from opencog.atomspace import fromtypes

class TestAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        self.counter = 1

    def run(self,atomspace):
        atomspace.add_node(types.ConceptNode, "testnode"+str(self.counter))
        print "I am running and the atomspace contains: "
        atomspace.print_list()
        self.counter+=1


