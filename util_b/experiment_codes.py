from util_b.general_util import BlendingLoggerForDebug

__author__ = 'DongMin Kim'

import os.path
import sys

import logging
import subprocess

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.bindlink import bindlink
from opencog.scheme_wrapper \
    import load_scm, scheme_eval, scheme_eval_h, __init__

# Remote debug is not working due to threading issue.
# Py_Initialze() method must be called by out of cogserver.
"""
class PyCharmDebugServer:
    def __init__(self):
        pass

    def start(self):

        sys.path.append("/usr/local/lib/python2.7/pycharm-debug.egg")

        BlendingLoggerForDebug().log('Python %s on %s' % (sys.version, sys.platform))
        BlendingLoggerForDebug().log('remote debugging')

        import pydevd

        pydevd.settrace(
            'localhost', port=19001, stdoutToServer=True, stderrToServer=True
        )
"""

class ExperimentCodes:
    def __init__(self, a):
        self.a = a

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def execute(self):
        # DEBUG: Run remote debug server for PyCharm remote debugging.
        # debug = PyCharmDebugServer()
        # debug.start()
        pass
