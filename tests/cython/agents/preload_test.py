import opencog.cogserver

print("Preloaded " + __name__)

class PreloadTestAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        pass

    def run(self, atomspace):
        print("running agent from preloaded file")
