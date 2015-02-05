#! /usr/bin/env python

"""
Checks the execution time of repeated calls to the Scheme API from Python

Runs an empty Scheme command NUMBER_OF_ITERATIONS times and displays the
total execution time
"""

__author__ = 'Cosmo Harrigan'

NUMBER_OF_ITERATIONS = 50000

from opencog.atomspace import AtomSpace, TruthValue, types, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__

atomspace = AtomSpace()
__init__(atomspace)

data = ["opencog/atomspace/core_types.scm",
        "opencog/scm/utilities.scm"]

for item in data:
    load_scm(atomspace, item)


def test_operation():
    for i in range(NUMBER_OF_ITERATIONS):
        scheme_eval(atomspace, '(+ 2 2)')

import timeit
elapsed = timeit.timeit("test_operation()",
                        setup="from __main__ import test_operation",
                        number=1)

print "{0} seconds elapsed performing {1} repeated calls = {2} calls / sec".\
    format(elapsed, NUMBER_OF_ITERATIONS, NUMBER_OF_ITERATIONS / elapsed)
