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

Module first parses all premises and conclusions with RelEx and RelEx2Logic.
Saves them as atomspace_inputs, desired and undesired outputs, respectively.
After all conclusions have been parsed, it loads the inputs again in the
CogServer and runs PLN on them.
"""

from opencog.cogserver import MindAgent
from client.client import *
from test_syllogisms import set_up_atomspace
from pln.chainers import Chainer


class SyllogismAgent(MindAgent):
    def __init__(self):
        self.chainer = None

    def create_chainer(self, local_atomspace):
        self.chainer = Chainer(local_atomspace,
                               agent=self,
                               stimulateAtoms=False,
                               # can be set to True to use AA on syllogisms
                               preferAttentionalFocus=False,
                               allow_output_with_variables=True,
                               delete_temporary_variables=True)
        # add rules here
        # self.chainer.add_rule()

    def run(self, local_atomspace):
        if self.chainer is None:
            self.create_chainer(local_atomspace)
            print("PLN Chainer created.")
            return

        print("PLN continuing.")

        result = self.chainer.forward_step()
        return result

num_steps = 10
file_name = "syllogisms.txt"
path = "../opencog/python/pln/examples/relex2logic/syllogisms/" \
       "syllogism_r2l_pln_pipeline"
name = "SyllogismAgent"

stop_agent_loop()
load_python_agent(path)
start_python_agent(path, name)

with open(file_name, "r") as f:
    atomspace_inputs = ""
    desired_outputs = ""
    undesired_outputs = ""
    for line in f:
        # clears atomspace
        clear_atomspace()
        line = line.strip("\n")

        # runs the agent when desired or undesired outputs have been saved
        if line == "" and (undesired_outputs or desired_outputs):
            # put inputs in CogServer so PLN can reason on them
            scheme(atomspace_inputs)
            current_atomspace_content = dump_atomspace_scheme()
            for t in range(0, num_steps):
                # run PLN
                step_python_agent(path, name)
                current_atomspace_content = dump_atomspace_scheme()
            #TODO: compare current_atomspace_content with outputs

        # empty line after syllogism causes values to be reset
        elif line == "":
            atomspace_inputs = ""
            desired_outputs = ""
            undesired_outputs = ""
            conclusions_parsed = False
            continue
        # comments are skipped
        elif line.startswith("#"):
            continue
        print line
        # removes a., b., |-, or |/-; parses string with RelEx & RelEx2Logic
        scheme("""(relex-parse "{0}")""".format(" ".join(line.split(" ")[1:])))
        scheme("(delete-sentences)")
        print(dump_atomspace_scheme())
        # saves inuts
        if line.startswith("a") or line.startswith("b"):
            atomspace_inputs += dump_atomspace_scheme()
        # saves desired outputs
        elif line.startswith("|-"):
            desired_outputs += dump_atomspace_scheme()
        # saves undesired outputs
        elif line.startswith("|/-"):
            undesired_outputs += dump_atomspace_scheme()
