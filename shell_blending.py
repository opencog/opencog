__author__ = 'DongMin Kim'

import sys
from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.utilities import *
from opencog.logger import *

from blender.random_blender import *
import blending_util

from experiment_codes import ExperimentCodes

# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)
        self._make_blend_target_for_debug()

        self.experiment_codes_inst = ExperimentCodes(self.a)
        self.random_blending_inst = RandomBlender(self.a)

    def __del__(self):
        self._delete_blend_target_for_debug()

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def get_atomspace_for_debug(self):
        return self.a

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

    def _backup_temp_link_list_for_debug(self, node_name_list):
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

    def _restore_temp_link_list_for_debug(self, info_list):
        for info in info_list:
            self.a.add_link(
                info[0],
                [info[1], info[2]],
                info[3]
            )

    def call_experiment_functions(self):
        self.experiment_codes_inst.execute()

    def run(self):
        log.warn("Start ShellBlending")

        self.call_experiment_functions()

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            exec_result = self.random_blending_inst.blend()

            if exec_result != 0:
                print "Error in blending class."
                break

            log.warn("Input n to preserve temp links, or delete.")
            is_delete_temp_link = raw_input()
            if is_delete_temp_link != 'n' or is_delete_temp_link != 'N':
                link_list_backup = \
                    self._backup_temp_link_list_for_debug(
                        [
                            "BlendTarget",
                            "InputSpace0", "InputSpace1", "GenericSpace"
                        ]
                    )

            log.warn("Input n to stop, or continue.")
            is_stop = raw_input()

            if is_delete_temp_link != 'n' or is_delete_temp_link != 'N':
                self._restore_temp_link_list_for_debug(link_list_backup)

            if is_stop == 'n' or is_stop == 'N':
                break

# Log will be written to opencog.log in the current directory.
log.set_level('WARN')
log.use_stdout()

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
inst.__del__()

# DEBUG: To keep program in running while view my result of coding.
raw_input("Press enter to exit\n")
