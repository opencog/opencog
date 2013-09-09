__author__ = 'cosmo'

from unittest import TestCase
from opencog.pymoses import moses


class MosesTest(TestCase):

    def setUp(self):
        pass#self.moses = moses()

    def tearDown(self):
        pass#del self.moses

    def test_run_moses(self):
        import tempfile

        self.moses = moses()
        #input_file = tempfile.NamedTemporaryFile()
        output_file = tempfile.NamedTemporaryFile()

        temp_input = "-i /home/cosmo/xor.txt "

        self.moses.run(temp_input + "-o " + output_file.name + " -c 1")
        output = output_file.file.read()

        #self.moses.run("--version")
        # assert something






