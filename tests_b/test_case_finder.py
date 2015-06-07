from util_b.general_util import BlLogger, get_class, \
    get_class_by_split_name, BlConfig

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from tests_b.paul_sally.paul_sally import PaulSallyExample
from tests_b.debate_with_kant.debate_with_kant import DebateWithKantExample

from opencog.logger import log

# Make several test cases for debug.
class TestCaseFactory:
    def __init__(self, a):
        self.a = a
        self.__make_default_concept()

        self.test_case_list = [
            PaulSallyExample,
            DebateWithKantExample
        ]

        self.test_case_count = len(self.test_case_list)

    def __make_default_concept(self):
        # Dummy truth values.
        self.atom_tv = TruthValue(1, 1)
        self.link_tv = TruthValue(1, 1)

        # - Base Concepts
        # Make space & frame concept.
        self.a_space = ConceptNode("Space", self.atom_tv)
        self.a_frame = ConceptNode("Frame", self.atom_tv)

    def make(self, id_or_name=None):
        if BlConfig().get('Example', 'EXAMPLE_LOAD') == 'False':
            return

        if id_or_name is None:
            id_or_name = BlConfig().get('Example', 'EXAMPLE')

        if type(id_or_name) is str:
            for test_case in self.test_case_list:
                if str(test_case).find(id_or_name) != -1:
                    maker_inst = test_case(self.a)
                    maker_inst.make()
        else:
            maker_inst = self.test_case_list[id_or_name](self.a)
            maker_inst.make()

    def make_all(self):
        for test_case in self.test_case_list:
            maker_inst = test_case(self.a)
            maker_inst.make()
