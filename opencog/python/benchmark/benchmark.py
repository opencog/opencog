"""
Python binding benchmarks

Displays the number of operations per second achieved under various scenarios
utilizing the Python bindings that wrap the AtomSpace using Cython

Results can be compared with the more comprehensive benchmarking code in
the opencog/benchmark directory.

Example results on November 4, 2014:
Add nodes
  Items per second: 166330
Add nodes and create a complete (fully-connected) graph
  Items per second: 162585
"""

import time
from opencog.atomspace import AtomSpace, TruthValue, Handle, types

__author__ = 'Cosmo Harrigan'


def test_1(a, n):
    """Add n nodes in atomspace a and returns the number of items processed"""
    for i in range(0, n):
        a.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n


def test_2(a, n):
    """Add n nodes and create a complete (fully-connected) graph in atomspace
    a and returns the number of items processed
    """
    offset = a.add_node(types.ConceptNode, "Starting handle offset")
    offset = offset.h.value()

    for i in range(1, n+1):
        a.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
        for j in range(1, i):
            a.add_link(types.HebbianLink,
                       [Handle(i + offset), Handle(j + offset)],
                       TruthValue(.2, .3))

    # Number of vertices plus number of edges in a fully connected graph
    return n + (n**2 - n) / 2


def do_test(test, name, n):
    """Runs test on n nodes and prints the test name and performance"""
    a = AtomSpace()
    start = time.time()
    ops = test(a, n)
    finish = time.time()
    print name
    print "  Items per second: {0:.0f}".format(ops / (finish - start))

tests = [test_1, test_2]
test_names = ["Add nodes",
              "Add nodes and create a complete (fully-connected) graph"]
do_test(tests[0], test_names[0], 1000000)
do_test(tests[1], test_names[1], 1000)
