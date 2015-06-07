import opencog.cogserver
from opencog.utilities import *
from blender_b.blender_finder import BlenderFinder
from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import BlLogger

__author__ = 'DongMin Kim'


# Perform Conceptual Blending.
class BlendingAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.run_by_server = True
        self.a = None

        self.blender_finder = None
        self.blender_inst = None

        self.config = None

    def __initialize(self, a):
        # Agent was already loaded.
        if self.a is not None:
            return
        self.a = a
        initialize_opencog(self.a)

        self.experiment_codes = ExperimentCodes(self.a)
        self.experiment_codes.init_hook()

        self.blender_finder = BlenderFinder(self.a)

    def __finalize(self):
        self.experiment_codes.final_hook()

    def run(self, a):
        # BlLogger().log("Start BlendingAgent")
        self.__initialize(a)

        self.blender_inst = self.blender_finder.get_blender()
        self.blender_inst.blend()

        self.__finalize()
        # BlLogger().log("Finish BlendingAgent")
