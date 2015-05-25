__author__ = 'DongMin Kim'

import os.path
import sys

import logging
import subprocess

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.bindlink import bindlink
from opencog.type_constructors \
    import ConceptNode, TypeNode, VariableNode, \
    UnorderedLink, MemberLink, InheritanceLink
from opencog.scheme_wrapper \
    import load_scm, scheme_eval, scheme_eval_h, __init__

sys.path.append(os.path.expanduser("~/opencog/opencog/python"))
import web.api.restapi

from util import blending_util


# Note: Divided to standalone class because I'll remove
class RESTAPILoader:
    def __init__(self, atomspace):
        self.a = atomspace

    def run(self):
        # To avoid debug messages of restapi.
        import logging

        logging.basicConfig(level=logging.CRITICAL)
        restapi = web.api.restapi.Start()
        restapi.run("", self.a)
        return 0


class ExperimentCodes:
    def __init__(self, atomspace):
        self.a = atomspace
        self.a_blend_target = self.make_blend_target_for_debug()
        self.link_list_backup = []

    # DEBUG: Make temporary concept - To define which node is target to blend.
    def make_blend_target_for_debug(self):
        return ConceptNode(blending_util.BLEND_TARGET_NODE_NAME)

    def delete_blend_target_for_debug(self):
        l_link_list = self.a.get_atoms_by_target_atom(
            types.Link, self.a_blend_target
        )

        for link in l_link_list:
            self.a.remove(link)

        self.a.remove(self.a_blend_target)

    def backup_debug_link_list(self):
        node_name_list = [
            "BlendTarget",
            "InputSpace0",
            "InputSpace1",
            "GenericSpace"
        ]
        self.link_list_backup = []

        for name in node_name_list:
            dst_node = self.a.get_atoms_by_name(types.Node, name)[0]
            get_target_link = \
                self.a.get_atoms_by_target_atom(types.Link, dst_node)

            for link in get_target_link:
                xget_target_link_node = self.a.xget_outgoing(link.h)
                for src_node in xget_target_link_node:
                    if src_node.h != dst_node.h:
                        self.link_list_backup.append(
                            (link.t, src_node, dst_node, link.tv)
                        )
                self.a.remove(link)

    def restore_debug_link_list(self):
        for info in self.link_list_backup:
            self.a.add_link(
                info[0],
                [info[1], info[2]],
                info[3]
            )

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def get_atomspace_for_debug(self):
        return self.a

    def execute(self):
        # DEBUG: Run RESTAPI server automatically to see my atomspace.
        rest_api_loader = RESTAPILoader(self.a)
        exec_result = rest_api_loader.run()
        if exec_result != 0:
            print "Error in restapi class."
