__author__ = 'Cosmo Harrigan'

from unittest import TestCase
from opencog.pymoses import moses, MosesException, MosesCandidate


class MosesTest(TestCase):
    def setUp(self):
        self.moses = moses()

    def tearDown(self):
        del self.moses

    def test_run_xor_python(self):
        input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        output = self.moses.run(input=input_data, python=True)
        self.assertIsInstance(output[0], MosesCandidate)
        self.assertEquals(output[0].score, 0)
        model = output[0].eval
        self.assertEquals(model([0, 0]), False)
        self.assertEquals(model([1, 1]), False)
        self.assertEquals(model([0, 1]), True)
        self.assertEquals(model([1, 0]), True)

    def test_run_xor_combo(self):
        input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        output = self.moses.run(input=input_data, args="-c 1")
        self.assertEquals(len(output), 1)
        self.assertIsInstance(output[0], MosesCandidate)
        self.assertEquals(output[0].score, 0)
        self.assertEquals(output[0].program, "and(or(!$1 !$2) or($1 $2))")

    def test_run_majority_python(self):
        output = self.moses.run(args="-H maj -c 2", python=True)
        self.assertIsInstance(output[0], MosesCandidate)
        self.assertEquals(output[0].score, 0)
        model = output[0].eval
        self.assertEquals(model([0, 1, 0, 1, 0]), False)  # Returns: False
        self.assertEquals(model([1, 1, 0, 1, 0]), True)  # Returns: True
        self.assertIsInstance(output[1], MosesCandidate)
        self.assertEquals(output[1].score, -1)

    def test_run_raise(self):
        self.assertRaises(MosesException, self.moses.run, args="-c")

    def test_run_manually(self):
        self.moses.run_manually("--version")

    def test_run_manually_raise(self):
        self.assertRaises(MosesException, self.moses.run_manually, args="-c")
