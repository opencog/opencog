__author__ = 'DongMin Kim'

import opencog.cogserver
from opencog.utilities import *

from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import *
from util_b.blending_util import *
from blender_b.blender_factory import BlenderFactory
from tests_b.test_case_factory import TestCaseFactory


# Perform Conceptual Blending.
class BlendingAgent(opencog.cogserver.MindAgent):
    # run_by_server = True
    a = None

    experiment_codes_inst = None
    test_case_factory = None

    blender_factory = None
    blender_inst = None

    config = None

    def __initialize(self, a):
        # BlendingLoggerForDebug().log("Start BlendingAgent")
        self.a = a

        if BlendingConfigLoader().is_use_blend_target:
            BlendTargetCtlForDebug().a = self.a
            BlendTargetCtlForDebug().make_blend_target()
            BlendTargetCtlForDebug().restore_debug_link_list()

        initialize_opencog(self.a)

        self.blender_factory = BlenderFactory(self.a)
        self.test_case_factory = TestCaseFactory(self.a)

        self.experiment_codes_inst = ExperimentCodes(self.a)
        self.experiment_codes_inst.execute()

    def __finalize(self):
        if BlendingConfigLoader().is_use_blend_target:
            BlendTargetCtlForDebug().backup_debug_link_list()
        # BlendingLoggerForDebug().log("Finish BlendingAgent")

    def __blender_select(self, id_or_name=None):
        if BlendingConfigLoader().is_use_config_file() is True:
            id_or_name = BlendingConfigLoader().get('Blender', 'BLENDER')

        """
        else:
            if id_or_name is None:
                self.blender_factory.print_blender_list()
                id_or_name = self.blender_factory.ask_to_user()
        """

        self.blender_inst = self.blender_factory.get_blender(id_or_name)

    def __test_case_select(self, id_or_name=None):
        if BlendingConfigLoader().get('Example', 'EXAMPLE_LOAD') == 'False':
            return

        if BlendingConfigLoader().is_use_config_file() is True:
            id_or_name = BlendingConfigLoader().get('Example', 'EXAMPLE')

        """
        else:
            if id_or_name is None:
                self.test_case_factory.print_test_case_list()
                id_or_name = self.test_case_factory.ask_to_user()
        """

        self.test_case_factory.make(id_or_name)

    def run(self, a):
        self.__initialize(a)

        self.__blender_select()
        self.__test_case_select()

        self.blender_inst.blend()

        if self.blender_inst.get_last_status() != 0:
            BlendingLoggerForDebug().log('Error in blending class.')

        self.__finalize()
