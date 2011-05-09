import opencog

class TestAgent(opencog.MindAgent):

    def run(self,a):
        print "I am running and the atomspace contains:"
        a.print_list()

