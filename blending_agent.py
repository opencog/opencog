__author__ = 'DongMin Kim'

import opencog
import opencog.cogserver
from opencog.utilities import *

from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import *
from util_b.blending_util import *
from blender_b.blender_finder import BlenderFactory
from tests_b.test_case_factory import TestCaseFactory


# Perform Conceptual Blending.
class BlendingAgent(opencog.cogserver.MindAgent):
    def __init__(self):
        self.run_by_server = True
        self.a = None

        self.experiment_codes_inst = None
        self.test_case_factory = None

        self.blender_factory = None
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

        self.blender_factory = BlenderFactory(self.a)
        self.test_case_factory = TestCaseFactory(self.a)

        self.experiment_codes_inst = ExperimentCodes(self.a)
        self.experiment_codes_inst.execute()

    def __finalize(self):
        if BlConfig().is_use_blend_target:
            BlendTargetCtlForDebug().backup_debug_link_list()
        # BlLogger().log("Finish BlendingAgent")

    def __blender_select(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get('Blender', 'BLENDER')

        self.blender_inst = self.blender_factory.get_blender(id_or_name)

    def __test_case_select(self, id_or_name=None):
        if BlConfig().get('Example', 'EXAMPLE_LOAD') == 'False':
            return

        if id_or_name is None:
            id_or_name = BlConfig().get('Example', 'EXAMPLE')

        self.test_case_factory.make(id_or_name)

    def run(self, a):
        self.__initialize(a)

        self.__blender_select()
        self.__test_case_select()

        self.blender_inst.blend()

        # if self.blender_inst.get_last_status() != 0:
        #    BlLogger().log('Error in blending class.')

        self.__finalize()
