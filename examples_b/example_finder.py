from examples_b.paul_sally.paul_sally import PaulSallyExample
from examples_b.debate_with_kant.debate_with_kant import DebateWithKantExample
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


# Make several test cases for debug.
class TestCaseLoader:
    def __init__(self, a):
        self.a = a

        self.test_cases = {
            PaulSallyExample.__name__: PaulSallyExample,
            DebateWithKantExample.__name__: DebateWithKantExample
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'EXAMPLE_LOAD': 'True',
            'EXAMPLE': 'PaulSallyExample'
        }
        BlConfig().make_default_config(str(self), default_config)

    def make(self, id_or_name=None):
        config = BlConfig().get_section(str(self))

        if config is None or config.get('EXAMPLE_LOAD') == 'False':
            return

        if id_or_name is None:
            id_or_name = config.get('EXAMPLE')

        test_case = self.test_cases.get(str(id_or_name))
        if test_case is not None:
            test_case(self.a).make()
        else:
            raise UserWarning('Test case not found.')

    def make_all(self):
        for test_case in self.test_cases.values():
            test_case(self.a).make()
