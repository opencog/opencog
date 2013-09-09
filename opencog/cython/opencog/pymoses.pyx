__author__ = 'cosmo'

'''
Wrapper for MOSES. Uses the C++ moses_exec function to access MOSES functionality.

Options for use:
    1. Within the CogServer, from an embedded MindAgent
    2. Within the CogServer, from the interactive python shell
    3. In your Python IDE or interpreter. You need to ensure that your path includes '{PROJECT_BINARY_DIR}/opencog/cython'

Loading the module:
    from opencog.pymoses import moses
    moses = moses()

To view the available arguments:
    moses.run("")

Example usage: @todo: Update this to the enhanced version and add python doc
    moses.run("--version")
    moses.run("-H pa -c 1")
    moses.run("-i input.txt -c 1 -o output.txt")

    XOR example with Python:

    output = moses.run(input = [[0,0,0],[1,1,0],[1,0,1],[0,1,1]], args = "--python 1")
    program = output[0].program
    exec program
    moses_eval([0, 1])

    Returns: True

Needed improvements:
    @todo Make moses_exec return the combo program of the best candidate solution and expose it to the Cython wrapper
          so that it can be wrapped in a GroundedSchemaNode and added to the AtomSpace.
    @todo Implement an option to use a Python function as the scoring function
'''

from libc.stdlib cimport malloc, free
from collections import namedtuple
import shlex
import tempfile
import csv

class MosesException(Exception):
    pass

cdef class moses:
    def __init__(self):
        pass

    def __dealloc__(self):
        pass

    # @todo remove this test method
    def test(self, args = ""):
        test_input = [['out','a','b'],
                      [0,0,0],
                      [1,1,0],
                      [1,0,1],
                      [0,1,1]]

        # @todo test functionality to allow an easy-to-access method to be created and returned or accessed based on the best candidate solution:
        #args += " --python 1"
        #python_output = self.run(input = test_input, args = args)
        #exec python_output[0].program
        #moses_solution = moses_eval @todo How to make moses_eval visible to Cython after exec?
        #return moses_solution

        return self.run(input = test_input, args = args)

    def run(self, input = None, args = ""):
        # Create temporary files for sending input/output to the moses_exec function
        if input is not None:
            input_file = tempfile.NamedTemporaryFile()

            input_file_builder = csv.writer(input_file, delimiter = ',')
            input_file_builder.writerows(input)

            input_file.flush()

        #temp_input_filename = "/home/cosmo/xor.txt"

        output_file = tempfile.NamedTemporaryFile()

        # Process the argument list for moses_exec
        _args_list = []
        if input is not None:
            _args_list.extend(['-i', input_file.name])
        _args_list.extend(['-o', output_file.name])
        _args_list.extend(shlex.split(args))
        #@todo
        print "Debug: args_list = "
        print _args_list
        self._run_args_list(_args_list)

        # Process the output file
        output = output_file.file.read()

        Candidate = namedtuple("Candidate", "score program program_type")
        candidates = []

        if output is None:
            # @todo raise error
            raise MosesException('Error: No output file was obtained from MOSES. Check to make sure the input file and arguments provided were valid.')

        # Python output
        # @todo fix iostream_combo.h, so that it doesn't add the extra comma at the end of the python program, on line 69, perhaps by adding a check to see if number_of_children > 1
        elif output.splitlines()[0].startswith('#!/usr/bin/python'): # @todo modify the python output parser to not put a blank line at the beginning
            # The Python header is declared in opencog/learning/moses/moses/types.h (ostream_combo_tree_composite_pbscore_python)
            python_header = "#!/usr/bin/python"

            output_list = [python_header + "\n" + element for element in output.split(python_header)[1:]]

            for candidate in output_list: #output.split("#!/usr/bin/python")[1:]:
                program = candidate #.splitlines()
                program = program.rpartition(',')[0] # @todo fix opencog/comboreduct/combo/iostream_combo.h (ostream_combo_it) to remove the unneeded trailing comma that is inserted by the Python formatter
                candidates.append(Candidate(score = None, program = program, program_type = "python")) # @todo add score for python programs

        # Combo output
        else:
            output_list = [element[:-1] for element in output.splitlines()]

            for candidate in output_list:
                score = candidate.partition(' ')[0]
                program = candidate.partition(' ')[2]
                candidates.append(Candidate(score = score, program = program, program_type = "combo"))

        return candidates

    def run_manually(self, args=""):
        self._run_args_list(args)

    def _run_args_list(self, args_list):
        args_list.insert(0, "moses")
        cdef char **c_argv
        args_list = [bytes(x) for x in args_list]
        c_argv = <char**>malloc(sizeof(char*) * len(args_list))
        for idx, s in enumerate(args_list):
            c_argv[idx] = s
        try:
            moses_exec(len(args_list), c_argv)
        except RuntimeError, ex:
            print "Exception occurred when calling MOSES:\n" + ex.message
            # @todo raise error
        finally:
            free(c_argv)