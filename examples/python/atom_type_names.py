"""
Example of how to obtain atom type names and atom type IDs in Python
"""

__author__ = 'Cosmo Harrigan'

from opencog.atomspace import AtomSpace, TruthValue, types, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__

atomspace = AtomSpace()
__init__(atomspace)

data = ["opencog/atomspace/core_types.scm",
        "opencog/scm/utilities.scm"]

for item in data:
    load_scm(atomspace, item)

atom = atomspace.add(types.ConceptNode, "Frog #1")

# To get one type name
print get_type_name(3) + '\n'

# To get one atom's type name
print get_type_name(atom.type) + '\n'

# Get a list of all possible type names and numbers
for key, value in sorted(types.__dict__.iteritems()):
    if '__' not in key:
        print key, value
