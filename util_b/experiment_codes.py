from tests_b.test_case_finder import TestCaseLoader
from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class ExperimentCodes:
    def __init__(self, a):
        self.a = a

    def test_ure(self):
        pass

    def foo(self):
        self.test_ure()

    def init_hook(self):
        # Remove all exist atoms to focus debug only blending agent.
        if BlConfig().get('General', 'AGENT_MODE') == 'Debug':
            self.a.clear()

        if BlConfig().is_use_blend_target:
            BlendTargetCtlForDebug().a = self.a
            BlendTargetCtlForDebug().make_blend_target()
            BlendTargetCtlForDebug().restore_debug_link_list()

        test_case_finder = TestCaseLoader(self.a)
        test_case_finder.make()

    def final_hook(self):
        if BlConfig().is_use_blend_target:
            BlendTargetCtlForDebug().backup_debug_link_list()

"""
import os.path
import sys
import subprocess

from opencog.scheme_wrapper import *

# Remote debug is not working due to threading issue.
# Py_Initialize() method must be called by out of cogserver.
class PyCharmDebugServer:
    def __init__(self):
        pass

    def start(self):

        sys.path.append("/usr/local/lib/python2.7/pycharm-debug.egg")

        BlLogger().log('Python %s on %s' % (sys.version, sys.platform))
        BlLogger().log('remote debugging')

        import pydevd

        pydevd.settrace(
            'localhost', port=19001, stdoutToServer=True, stderrToServer=True
        )
"""
