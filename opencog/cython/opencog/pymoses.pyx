"""
Wrapper for MOSES. Uses the C++ moses_exec function to access MOSES
functionality.

Options for using the pymoses wrapper:
1. Within the CogServer, from an embedded MindAgent
2. Within the CogServer, from the interactive Python shell
3. In your Python IDE or interpreter. You need to ensure that
   your path includes '{PROJECT_BINARY_DIR}/opencog/cython'

Loading the module:

from opencog.pymoses import moses
moses = moses()

Example usage of run:

Example #1: XOR example with Python output and Python input

input_data = [[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
output = moses.run(input=input_data, python=True)
print output[0].score # Prints: 0
model = output[0].eval
model([0, 1])  # Returns: True
model([1, 1])  # Returns: False

Example #2: Run the majority demo problem, return only one candidate, and use
Python output

output = moses.run(args="-H maj -c 1", python=True)
model = output[0].eval
model([0, 1, 0, 1, 0])  # Returns: False
model([1, 1, 0, 1, 0])  # Returns: True

Example #3: Load the XOR input data from a file, return only one candidate,
and use Combo output

output = moses.run(args="-i /path/to/input.txt -c 1")
combo_program = output[0].program
print combo_program  # Prints: and(or(!$1 !$2) or($1 $2))

Example usage of run_manually:

moses.run_manually("-i input.txt -o output.txt")

@todo Implement an option to use a Python function as the MOSES scoring
function
"""

__author__ = 'Cosmo Harrigan'

from libc.stdlib cimport malloc, free
import shlex
import tempfile
import csv

class MosesException(Exception):
    pass

class MosesCandidate(object):
    def __init__(self, score = None, program = None, program_type = None):
        self.score = score
        self.program = program
        self.program_type = program_type

    def eval(self, arglist):
        if self.program_type != "python":
            raise MosesException('Error: eval method is only defined for '
                                 'candidates with program_type of python.')
        if len(arglist) == 0:
            raise MosesException('Error: eval method requires a list of input '
                                 'values.')

        namespace = {}
        exec self.program in namespace
        return namespace.get('moses_eval')(arglist)

cdef class moses:
    def run(self, input = None, args = "", python = False):
        """
        Invokes MOSES in supervised learning mode to learn candidate solutions
        for a given training set.

        Parameters:
            input (list of lists) - training data for regression [optional]
                Example: input=[[0, 0, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
            args (string) - arguments for MOSES (see MOSES documentation)
                            [optional]
            python (bool) - if True, return Python instead of Combo [optional]
        Either input or args must be provided, otherwise, MOSES would have no
        input

        Output:
        Returns a collection of candidates as MosesCandidate objects.
        Each MosesCandidate contains:
            score (int)
            program (string)
            program_type (string - Enumeration: python, combo)
            eval (Runnable method - Only valid for program_type python) Run
            this method to evaluate the model on new data.
        """
        if (input is None or input == "") and args == "":
            raise MosesException('Error: input and args cannot both be empty. '
                                 'You should pass your input using the input '
                                 'parameter, or refer to it in the args '
                                 'parameter.')

        # Create temporary files for sending input/output to the moses_exec
        # function
        if input is not None:
            input_file = tempfile.NamedTemporaryFile()
            input_file_builder = csv.writer(input_file, delimiter = ',')
            input_file_builder.writerows(input)
            input_file.flush()

        output_file = tempfile.NamedTemporaryFile()

        # Process the argument list for moses_exec
        _args_list = []

        if input is not None:
            _args_list.extend(['-i', input_file.name])
        if python:
            _args_list.extend(['--python', '1'])

        _args_list.extend(['-o', output_file.name])
        _args_list.extend(shlex.split(args))

        self._run_args_list(_args_list)
        # Process the output file
        output = output_file.file.read()

        candidates = []
        # Python header declared in opencog/learning/moses/moses/types.h
        # (ostream_combo_tree_composite_pbscore_python)
        python_header = "#!/usr/bin/env python"

        if len(output) == 0:
            raise MosesException('Error: No output file was obtained from '
                                 'MOSES. Check to make sure the input file '
                                 'and arguments provided were valid.')

        # Python output
        elif output.splitlines()[0].startswith(python_header):
            output_list = [python_header + "\n" + element for
                           element in output.split(python_header)[1:]]

            for candidate in output_list:
                program = candidate
                # @todo Fix opencog/comboreduct/combo/iostream_combo.h
                # (ostream_combo_it) to remove the unneeded trailing comma
                # that is inserted by the Python formatter
                if ',' in program:
                    program = program.rpartition(',')[0]
                if "#score: " in program:
                    score = int(program.split("#score: ")[1].splitlines()[0])
                else:
                    raise MosesException('Error: A score value was expected '
                                         'but not found in the Python '
                                         'program.')
                candidates.append(MosesCandidate(score = score,
                                                 program = program,
                                                 program_type = "python"))

        # Combo output
        else:
            output_list = [element for element in output.splitlines()]

            for candidate in output_list:
                score = int(candidate.partition(' ')[0])
                program = candidate.partition(' ')[2]
                candidates.append(MosesCandidate(score = score,
                                                 program = program,
                                                 program_type = "combo"))

        return candidates

    def run_manually(self, args=""):
        """
        Invokes MOSES without any extra integration.
        Useful for interacting with MOSES non-programatically via stdout.
        """
        if args == "":
            raise MosesException('Error: No arguments provided')
        self._run_args_list(shlex.split(args))

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
            if ex is None:
                ex = ""
            raise MosesException('Error: exception occurred when calling C++ '
                                 'MOSES. Exception message:\n' + ex.message)
        finally:
            free(c_argv)
