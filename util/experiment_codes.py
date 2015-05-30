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

import opencog.logger
from opencog.logger import log

from util import blending_util

"""
class PyCharmDebugServer:
    def __init__(self):
        sys.path.append("/usr/local/lib/python2.7/pycharm-debug.egg")
        import pydevd

        pydevd.settrace(
            'localhost', port=19001, stdoutToServer=True, stderrToServer=True
        )
"""

class ExperimentCodes:
    def __init__(self, atomspace, run_by_server):
        self.a = atomspace
        self.run_by_server = run_by_server

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def execute(self):
        # DEBUG: Run remote debug server for PyCharm remote debugging.
        # PyCharmDebugServer()
        pass
