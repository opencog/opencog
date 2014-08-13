__author__ = 'sebastian'

"""
Module to run RelEx and load the RelEx2Logic representation of a sentence
directly in Python

Expects input of the form seen in syllogisms.txt:
a. 1st premise
b. 2nd premise
|- Conclusion

Premises:
    - CogServer must be running
    - RelEx must be running
    - REST API must have been started in the CogServer (restapi.Start)
"""

from client.client import clear_atomspace, scheme, dump_atomspace_scheme
from test_syllogisms import set_up_atomspace

file_name = "syllogisms.txt"

with open(file_name, "r") as f:
    atomspace_inputs = ""
    expected_outputs = ""
    for line in f:
        line = line.strip("\n")
        if line == "":
            atomspace_inputs = ""
            expected_outputs = ""
            continue
        elif line.startswith("#"):
            continue
        print line
        clear_atomspace()
        # removes a., b. or |-, parses string with relex
        scheme("""(relex-parse "{0}")""".format(" ".join(line.split(" ")[1:])))
        scheme("(delete-sentences)")
        print(dump_atomspace_scheme())
        if line.startswith("a") or line.startswith("b"):
            atomspace_inputs += dump_atomspace_scheme()
        # if there is only one expected conclusion, chainer can be run
        # after the |- has been seen
        elif line.startswith("|-"):
            expected_outputs += dump_atomspace_scheme()
            atomspace = set_up_atomspace()
            # run chainer/agent either in Python or in the CogServer on the
            # atomspace_inputs, then check if the expected outputs have been
            # produced
