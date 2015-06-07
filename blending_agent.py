__author__ = 'DongMin Kim'

import opencog
import opencog.cogserver
from opencog.utilities import *

from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import *
from util_b.blending_util import *
from blender_b.blender_finder import BlenderFinder
from tests_b.test_case_finder import TestCaseFactory


# Perform Conceptual Blending.
class BlendingAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.run_by_server = True
        self.a = None

        self.experiment_codes_inst = None
        self.test_case_finder = None

        self.blender_finder = None
        self.blender_inst = None

        self.config = None

    def __initialize(self, a):
        # Agent was already loaded.
        if self.a is not None:
            return

        # BlLogger().log("Start BlendingAgent")
        self.a = a
        initialize_opencog(self.a)

        # Remove all exist atoms to focus debug only blending agent.
        if BlConfig().get('General', 'AGENT_MODE') == 'Debug':
            self.a.clear()

        if BlConfig().is_use_blend_target:
            BlendTargetCtlForDebug().a = self.a
            BlendTargetCtlForDebug().make_blend_target()
            BlendTargetCtlForDebug().restore_debug_link_list()

        self.blender_finder = BlenderFinder(self.a)
        self.test_case_finder = TestCaseFactory(self.a)

        self.experiment_codes_inst = ExperimentCodes(self.a)
        self.experiment_codes_inst.execute()

    def __finalize(self):
        if BlConfig().is_use_blend_target:
            BlendTargetCtlForDebug().backup_debug_link_list()
        # BlLogger().log("Finish BlendingAgent")

    def run(self, a):
        self.__initialize(a)

        self.test_case_finder.make()

        self.blender_inst = self.blender_finder.get_blender()
        self.blender_inst.blend()

        # if self.blender_inst.get_last_status() != 0:
        #    BlLogger().log('Error in blending class.')

        self.__finalize()
