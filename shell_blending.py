from blender.blender_factory import BlenderFactory
from tests.test_case_factory import TestCaseFactory

__author__ = 'DongMin Kim'

from opencog.atomspace import *
from opencog.utilities import *

from blender.random_blender import *
from util.experiment_codes import ExperimentCodes

# Perform Conceptual Blending.
class ShellBlending:
    """
    :type a: opencog.atomspace_details.AtomSpace
    :type blender_inst: blender.base_blender.BaseBlender
    """
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)

        self.experiment_codes_inst = ExperimentCodes(self.a)

        self.blender_factory = BlenderFactory(self.a)
        self.blender_inst = None

        self.test_case_factory = TestCaseFactory(self.a)

        # Log will be written to opencog.log in the current directory.
        log.set_level('WARN')
        log.use_stdout()

    def __del__(self):
        self.experiment_codes_inst.delete_blend_target_for_debug()

    def __ask_user_to_run_or_stop(self):
        # log.warn("Input n to preserve temp links, or delete.")
        # is_delete_temp_link = raw_input()
        is_preserve_debug_link = 'y'

        if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
            self.experiment_codes_inst.backup_debug_link_list()

        # self.experiment_codes_inst.print_atomspace_for_debug()

        log.warn("Input n to stop, or continue.")
        is_stop = raw_input()

        if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
            self.experiment_codes_inst.restore_debug_link_list()

        return is_stop

    def __blender_select(self, id_or_name=None):
        if id_or_name is None:
            self.blender_factory.print_blender_list()
            id_or_name = self.blender_factory.ask_to_user()

        self.blender_inst = self.blender_factory.get_blender(id_or_name)

    def __test_case_select(self, id_or_name=None):
        if id_or_name is None:
            self.test_case_factory.print_test_case_list()
            id_or_name = self.test_case_factory.ask_to_user()

        self.test_case_factory.make(id_or_name)

    def run(self):
        log.warn("Start ShellBlending")

        self.experiment_codes_inst.execute()

        # if no argument -> custom select.
        # if (0) or ('RandomBlender') -> pre-defined select.
        self.__blender_select('RandomBlender')

        # if no argument -> custom select.
        # if (0) or ('PaulSallyExample') -> pre-defined select.
        self.__test_case_select('DebateWithKantExample')

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            # self.blender_inst.blend()

            if self.blender_inst.get_last_status() != 0:
                print log.warn('Error in blending class.')
                break

            is_stop = self.__ask_user_to_run_or_stop()

            if is_stop == 'n' or is_stop == 'N':
                break
