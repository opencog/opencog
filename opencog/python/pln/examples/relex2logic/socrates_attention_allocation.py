from __future__ import print_function
from pln.examples.relex2logic import evaluation_to_member_agent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.examples.interactive_agent import InteractiveAgent
from attention_interface import *
from time import sleep

__author__ = 'Sebastian Ruder'

atomspace = AtomSpace()
__init__(atomspace)

coreTypes = "opencog/atomspace/core_types.scm"
utilities = "opencog/scm/utilities.scm"
data = "opencog/python/pln/examples/relex2logic/r2l-output-test.scm"

for item in [coreTypes, utilities, data]:
    load_scm(atomspace, item)

# Configurable parameters:
num_steps = 10                            # Number of time steps
output_filename = 'ecan-timeseries.csv'   # Output filename

timeseries = []

# Run num_steps cycles, capturing the contents of the attentional focus
# at each discrete timestep, and then export the dataset

all_atoms = atomspace.get_atoms_by_type(t=types.Atom)
print("Number of atoms in atomspace: %d" %len(all_atoms))
for atom in all_atoms:
    print(atom)

for t in range(0, num_steps):
    point_in_time = get_attentional_focus(timestep=t)
    timeseries.append(point_in_time)
    importance_diffusion()
    atoms = point_in_time.get_atoms()
    for atom in atoms:
        print(atom)
        print(atom.get_sti())

    print(point_in_time)
    sleep(.1)

write_timeseries(timeseries, output_filename)

all_atoms = atomspace.get_atoms_by_type(t=types.Atom)
print("Number of atoms in atomspace after inference: %d" %len(all_atoms))