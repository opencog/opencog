from opencog.type_constructors import *
from tests_b.test_case_finder import TestCaseLoader
from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig
from utility.util import pp

__author__ = 'DongMin Kim'


class ExperimentCodes:
    def __init__(self, a):
        self.a = a

    def test_ure(self):
        pass

    # TODO: Inherit chooser, decider, ... to BLEND?
    def __config_general_default(self):
        # blend
        ConceptNode("BLEND")
        # blend config
        ExecutionLink(
            SchemaNode("BLEND:config-format-version"),
            ConceptNode("BLEND"),
            ConceptNode("1")
        )
        ExecutionLink(
            SchemaNode("BLEND:execute-mode"),
            ConceptNode("BLEND"),
            ConceptNode("Release")
        )

    def __config_logger_default(self):
        # blend logger config
        ExecutionLink(
            SchemaNode("BLEND:log-level"),
            ConceptNode("Release"),
            ConceptNode("INFO")
        )
        ExecutionLink(
            SchemaNode("BLEND:log-level"),
            ConceptNode("Debug"),
            ConceptNode("WARN")
        )

        ExecutionLink(
            SchemaNode("BLEND:log-prefix"),
            ConceptNode("Release"),
            ConceptNode("[BlendingAgent]::")
        )
        ExecutionLink(
            SchemaNode("BLEND:log-prefix"),
            ConceptNode("Debug"),
            ConceptNode("[BA]==>")
        )

        ExecutionLink(
            SchemaNode("BLEND:log-postfix"),
            ConceptNode("Release"),
            ConceptNode("")
        )
        ExecutionLink(
            SchemaNode("BLEND:log-postfix"),
            ConceptNode("Debug"),
            ConceptNode("")
        )

    def __config_blender_default(self):
        ExecutionLink(
            SchemaNode("BLEND:blender"),
            ConceptNode("BLEND"),
            ConceptNode("RuleBlender")
        )

        InheritanceLink(
            ConceptNode("RuleBlender"),
            ConceptNode("BLEND")
        )
        InheritanceLink(
            ConceptNode("NoRuleBlender"),
            ConceptNode("BLEND")
        )

    def __config_atoms_chooser_default(self):
        ExecutionLink(
            SchemaNode("BLEND:atoms-chooser"),
            ConceptNode("BLEND"),
            ConceptNode("ChooseAll")
        )
        ExecutionLink(
            SchemaNode("BLEND:atoms-chooser"),
            ConceptNode("NoRuleBlender"),
            ConceptNode("ChooseNull")
        )

    def __config_blending_decider_default(self):
        ExecutionLink(
            SchemaNode("BLEND:blending-decider"),
            ConceptNode("BLEND"),
            ConceptNode("DecideBestSTI")
        )
        ExecutionLink(
            SchemaNode("BLEND:blending-decider"),
            ConceptNode("NoRuleBlender"),
            ConceptNode("DecideNull")
        )

    def __config_new_blend_atom_maker_default(self):
        ExecutionLink(
            SchemaNode("BLEND:new-blend-atom-maker"),
            ConceptNode("BLEND"),
            ConceptNode("MakeSimple")
        )

    def __config_link_connector_default(self):
        ExecutionLink(
            SchemaNode("BLEND:link-connector"),
            ConceptNode("BLEND"),
            ConceptNode("ConnectSimple")
        )

    def __config_chooser_atom_type_default(self):
        ExecutionLink(
            SchemaNode("BLEND:choose-atom-type"),
            ConceptNode("BLEND"),
            ConceptNode("Node")
        )

    def __config_chooser_least_count_default(self):
        ExecutionLink(
            SchemaNode("BLEND:choose-least-count"),
            ConceptNode("BLEND"),
            ConceptNode("2")
        )

    def __config_chooser_sti_min_default(self):
        ExecutionLink(
            SchemaNode("BLEND:choose-sti-min"),
            ConceptNode("ChooseInSTIRange"),
            ConceptNode("IMPORTANT")
        )

    def __config_chooser_sti_max_default(self):
        ExecutionLink(
            SchemaNode("BLEND:choose-sti-max"),
            ConceptNode("ChooseInSTIRange"),
            ConceptNode("NONE")
        )

    def __config_decider_result_atoms_count_default(self):
        ExecutionLink(
            SchemaNode("BLEND:decide-result-atoms-count"),
            ConceptNode("BLEND"),
            ConceptNode("2")
        )

    def __config_decider_sti_min_default(self):
        ExecutionLink(
            SchemaNode("BLEND:decide-sti-min"),
            ConceptNode("DecideBestSTI"),
            ConceptNode("IMPORTANT")
        )

    def __config_decider_sti_max_default(self):
        ExecutionLink(
            SchemaNode("BLEND:decide-sti-max"),
            ConceptNode("DecideBestSTI"),
            ConceptNode("NONE")
        )

    def __config_my_test_case(self):
        ConceptNode("my-test-case")
        InheritanceLink(
            ConceptNode("my-test-case"),
            ConceptNode("BLEND")
        )
        ExecutionLink(
            SchemaNode("BLEND:execute-mode"),
            ConceptNode("my-test-case"),
            ConceptNode("Debug")
        )

    def foo(self):
        # self.test_ure()
        self.__config_general_default()
        self.__config_logger_default()
        self.__config_blender_default()
        self.__config_atoms_chooser_default()
        self.__config_blending_decider_default()
        self.__config_new_blend_atom_maker_default()
        self.__config_link_connector_default()
        self.__config_chooser_atom_type_default()
        self.__config_chooser_least_count_default()
        self.__config_chooser_sti_min_default()
        self.__config_chooser_sti_max_default()
        self.__config_decider_result_atoms_count_default()
        self.__config_decider_sti_min_default()
        self.__config_decider_sti_max_default()
        self.__config_my_test_case()

    def init_hook(self):
        self.foo()
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
