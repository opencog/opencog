__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from tests.paul_sally.paul_sally import PaulSallyExample
from tests.debate_with_kant.debate_with_kant import DebateWithKantExample

from opencog.logger import log

# Make several test cases for debug.
class TestCaseFactory:
    def __init__(self, atomspace):
        self.a = atomspace
        self.__make_default_concept()

        self.test_case_list = [
            PaulSallyExample(self.a),
            DebateWithKantExample(self.a)
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
        log.warn('Please select test case number to use.')
        for i in range(self.test_case_count):
            test_case = self.test_case_list[i]
            log.warn(str(i) + ': ' + str(test_case))

    def ask_to_user(self):
        index = -1
        while (index < 0) or (index >= self.test_case_count):
            index = input()

        return index

    def make(self, id_or_name):
        if type(id_or_name) is str:
            for test_case in self.test_case_list:
                if str(test_case) == id_or_name:
                    test_case.make()
        else:
            self.test_case_list[id_or_name].make()

    def make_all(self):
        for test_case in self.test_case_list:
            test_case.make()
