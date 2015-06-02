from util_b.general_util import BlLogger, get_class, \
    get_class_by_split_name

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

    def print_test_case_list(self):
        BlLogger().log('Please select test case number to use.')
        for i in range(self.test_case_count):
            test_case = self.test_case_list[i]
            BlLogger().log(str(i) + ': ' + str(test_case))

    def ask_to_user(self):
        index = -1
        while (index < 0) or (index >= self.test_case_count):
            index = input()

        return index

    def make(self, id_or_name):
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
