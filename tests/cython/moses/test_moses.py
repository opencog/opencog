__author__ = 'Cosmo Harrigan'

from nose.tools import *
from opencog.pymoses import moses, MosesException, MosesCandidate


class TestMOSES:
    def setUp(self):
        self.moses = moses()

    def tearDown(self):
        del self.moses

    def test_run_xor_python(self):
        input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        output = self.moses.run(input=input_data, python=True)
        assert isinstance(output[0], MosesCandidate)
        assert output[0].score == 0
        model = output[0].eval
        assert not model([0, 0])
        assert not model([1, 1])
        assert model([0, 1])
        assert model([1, 0])

    def test_run_xor_combo(self):
        input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
        output = self.moses.run(input=input_data, args="-c 1")
        assert len(output) == 1
        assert isinstance(output[0], MosesCandidate)
        assert output[0].score == 0
        assert output[0].program == "and(or(!$1 !$2) or($1 $2))"

    def test_run_majority_python(self):
        output = self.moses.run(args="-H maj -c 2", python=True)
        assert isinstance(output[0], MosesCandidate)
        assert output[0].score == 0
        model = output[0].eval
        assert not model([0, 1, 0, 1, 0])
        assert model([1, 1, 0, 1, 0])
        assert isinstance(output[1], MosesCandidate)
        assert output[1].score == -1

    @raises(MosesException)
    def test_run_raise(self):
        assert raises(MosesException, self.moses.run(args="-c"))

    @raises(MosesException)
    def test_run_manually_raise(self):
        assert raises(MosesException, self.moses.run_manually(args="-c"))
