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
    run_by_server = True
    use_config_file = True
    a = None

    experiment_codes_inst = None
    test_case_factory = None

    blender_factory = None
    blender_inst = None

    config = None

    def __initialize(self, a):
        self.a = a

        self.use_config_file = BlendingConfigLoader().is_use_config_file()

        BlendTargetCtlForDebug().a = self.a
        BlendTargetCtlForDebug().make_blend_target()
        BlendTargetCtlForDebug().restore_debug_link_list()

        initialize_opencog(self.a)

        self.blender_factory = BlenderFactory(self.a)
        self.test_case_factory = TestCaseFactory(self.a)

        self.experiment_codes_inst = ExperimentCodes(self.a, self.run_by_server)
        self.experiment_codes_inst.execute()

    def __finalize(self):
        BlendTargetCtlForDebug().backup_debug_link_list()

    def __blender_select(self, id_or_name=None):
        if self.use_config_file is True:
            id_or_name = self.__get_config('BLENDING_BLENDER')
        else:
            if id_or_name is None:
                self.blender_factory.print_blender_list()
                id_or_name = self.blender_factory.ask_to_user()

        self.blender_inst = self.blender_factory.get_blender(id_or_name)

    def __test_case_select(self, id_or_name=None):
        if self.use_config_file is True:
            id_or_name = self.__get_config('BLENDING_EXAMPLE')
        else:
            if id_or_name is None:
                self.test_case_factory.print_test_case_list()
                id_or_name = self.test_case_factory.ask_to_user()

        self.test_case_factory.make(id_or_name)

    # TODO: link with global config in cogserver
    def __get_config(self, key):
        return BlendingConfigLoader().get(key)

    def __set_config(self, key, value):
        BlendingConfigLoader().set(key, value)

    def run(self, atomspace):
        # BlendingLoggerForDebug().log("Start BlendingAgent")
        self.__initialize(atomspace)

        self.__blender_select()
        self.__test_case_select()

        self.blender_inst.blend()

        if self.blender_inst.get_last_status() != 0:
            BlendingLoggerForDebug().log('Error in blending class.')

        self.__finalize()
        # BlendingLoggerForDebug().log("Finish BlendingAgent")
