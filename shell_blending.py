from blender.blender_factory import BlenderFactory

__author__ = 'DongMin Kim'

from opencog.atomspace import *
from opencog.utilities import *

from blender.random_blender import *
from util import blending_util
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
        self._make_blend_target_for_debug()

        self.experiment_codes_inst = ExperimentCodes(self.a)

        self.blender_factory = BlenderFactory(self.a)
        self.blender_inst = None

        # Log will be written to opencog.log in the current directory.
        log.set_level('WARN')
        log.use_stdout()

    def __del__(self):
        self._delete_blend_target_for_debug()

    # DEBUG: Make temporary concept - To define which node is target to blend.
    def _make_blend_target_for_debug(self):
        self.a_blend_target = ConceptNode(
            blending_util.BLEND_TARGET_NODE_NAME,
        )

    def _delete_blend_target_for_debug(self):
        # l_link_list = self.a.get_incoming(self.a_blend_target.h)
        l_link_list = self.a.get_atoms_by_target_atom(
            types.Link, self.a_blend_target
        )

        for link in l_link_list:
            self.a.remove(link)

        self.a.remove(self.a_blend_target)

    def __backup_debug_link_list(self, node_name_list):
        info_list = []

        for name in node_name_list:
            dst_node = self.a.get_atoms_by_name(types.Node, name)[0]
            get_target_link = \
                self.a.get_atoms_by_target_atom(types.Link, dst_node)

            for link in get_target_link:
                xget_target_link_node = self.a.xget_outgoing(link.h)
                for src_node in xget_target_link_node:
                    if src_node.h != dst_node.h:
                        info_list.append((link.t, src_node, dst_node, link.tv))
                self.a.remove(link)

        return info_list

    def __restore_debug_link_list(self, info_list):
        for info in info_list:
            self.a.add_link(
                info[0],
                [info[1], info[2]],
                info[3]
            )

    def __ask_user_to_run_or_stop(self):
        # log.warn("Input n to preserve temp links, or delete.")
        # is_delete_temp_link = raw_input()
        is_preserve_debug_link = 'y'
        link_list_backup = []
        if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
            link_list_backup = \
                self.__backup_debug_link_list(
                    [
                        "BlendTarget",
                        "InputSpace0", "InputSpace1", "GenericSpace"
                    ]
                )

        # self.print_atomspace_for_debug()

        log.warn("Input n to stop, or continue.")
        is_stop = raw_input()

        if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
            self.__restore_debug_link_list(link_list_backup)

        return is_stop

    def __blender_select(self, id_or_name=None):
        if id_or_name is None:
            self.blender_factory.print_blender_list()
            id_or_name = self.blender_factory.ask_to_user()

        self.blender_inst = self.blender_factory.get_blender(id_or_name)

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def get_atomspace_for_debug(self):
        return self.a

    def call_experiment_functions(self):
        self.experiment_codes_inst.execute()

    def run(self):
        log.warn("Start ShellBlending")

        self.call_experiment_functions()
        # self.__blender_select(0)
        self.__blender_select('RandomBlender')

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            self.blender_inst.blend()

            if self.blender_inst.get_last_status() != 0:
                print log.warn('Error in blending class.')
                break

            is_stop = self.__ask_user_to_run_or_stop()

            if is_stop == 'n' or is_stop == 'N':
                break
