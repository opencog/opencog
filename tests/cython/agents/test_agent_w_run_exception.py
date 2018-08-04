import opencog.cogserver

class TestRunExceptionAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        pass

    def run(self,atomspace):
        raise IndexError("This is a test exception")
