"""
Example of how to convert a set of atoms into a DOT graph description

Refer to README.md before use.
"""

from opencog.atomspace import AtomSpace, TruthValue, types, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
import dot

__author__ = 'Cosmo Harrigan'

atomspace = AtomSpace()
__init__(atomspace)

data = ["opencog/atomspace/core_types.scm",
        "opencog/scm/utilities.scm",
        "opencog/python/pln/examples/tuffy/smokes/smokes.scm"]

# Optionally, you could also include this file for a larger graph sample:
#   "opencog/python/pln/examples/tuffy/smokes/extra-data.scm"

for item in data:
    load_scm(atomspace, item)

# Define the set of atoms to include
atomset = atomspace.get_atoms_by_type(types.Atom)

# Obtain a description of the subhypergraph in the DOT graph description
# language
dot_output = dot.get_dot_representation(atomset)

print dot_output
