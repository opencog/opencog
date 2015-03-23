from unittest import TestCase

from opencog.agent_finder import find_subclasses
import opencog.cogserver

import test_agent

class HelperTest(TestCase):

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_find_agents(self):
        x = find_subclasses(test_agent, opencog.cogserver.MindAgent)
        self.assertEqual(len(x), 1)
        self.assertEqual(x[0][0], 'TestAgent')

