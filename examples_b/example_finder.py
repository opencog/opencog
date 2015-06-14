from examples_b.paul_sally.paul_sally import PaulSallyExample
from examples_b.debate_with_kant.debate_with_kant import DebateWithKantExample

__author__ = 'DongMin Kim'


# Make several test cases for debug.
class TestCaseLoader:
    def __init__(self, a):
        self.a = a

        self.test_cases = {
            PaulSallyExample.__name__: PaulSallyExample,
            DebateWithKantExample.__name__: DebateWithKantExample
        }

    def __str__(self):
        return self.__class__.__name__

    def make(self, id_or_name):
        test_case = self.test_cases.get(str(id_or_name))
        if test_case is not None:
            test_case(self.a).make()
        else:
            raise UserWarning('Test case not found.')

    def make_all(self):
        for test_case in self.test_cases.values():
            test_case(self.a).make()
