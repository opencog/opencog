#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Python binding benchmarks

Displays the number of operations per second achieved under various scenarios
utilizing the Python bindings that wrap the AtomSpace using Cython. Some of the
tests compare the evaluation of nodes using the Python Scheme bindings which
take much longer than their comparable Cython bindings.

Results can be compared with the more comprehensive benchmarking code in
the opencog/benchmark directory.

Example results (excerpt only):
--- OpenCog Python Benchmark -  2015-03-12 15:44:02.614126 ---

-- Testing Node Adds --

Add nodes - Cython
  Total time: 0.537
  Ops: 100,000
  Op time in µs: 5.37
  Ops per second: 186,219

Add nodes - Cython (500K)
  Total time: 2.792
  Ops: 500,000
  Op time in µs: 5.58
  Ops per second: 179,115

Add fully connected nodes
  Total time: 5.049
  Ops: 500,500
  Op time in µs: 10.09
  Ops per second: 99,132

...

"""

import argparse
import time, datetime
from opencog.atomspace import AtomSpace, TruthValue, Handle, types
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h
from opencog.bindlink import stub_bindlink, bindlink, validate_bindlink

__authors__ = 'Cosmo Harrigan, Curtis Faith'

class SmartFormatter(argparse.HelpFormatter):

    def _split_lines(self, text, width):
        # this is the RawTextHelpFormatter._split_lines
        if text.startswith('R|'):
            return text[2:].splitlines()  
        return argparse.HelpFormatter._split_lines(self, text, width)

# Setup the argument parser.
parser = argparse.ArgumentParser(description="OpenCog Python Benchmark",
        formatter_class=SmartFormatter)
verbosity_group = parser.add_mutually_exclusive_group()
verbosity_group.add_argument("-v", "--verbose", action="store_true", help="verbose output")
verbosity_group.add_argument("-c", "--columns", action="store_true", default=True, help="columnar output (default)")
#verbosity_group.add_argument("-q", "--quiet", action="store_true", help="minimal output")
test_group = parser.add_mutually_exclusive_group()
test_group.add_argument("-a", "--all",  default=False, action="store_true", help="run all tests")
test_group.add_argument("-t", "--test", type=str, default='spread',
                        choices=['spread','node', 'bindlink', 'traverse', 'scheme'],
                        help="R|Test to benchmark, where:\n"
                             "  spread    - a spread of tests across areas (default)\n"
                             "  node      - atomspace node operations \n"
                             "  bindlink  - all the bindlink forms\n"
                             "  traverse  - traversal of atomspace lists\n"
                             "  scheme    - scheme evaluation functions")
parser.add_argument("-i", "--iterations", metavar='N', type=int, default=10, help="iterations to average (default=10)")
args = parser.parse_args()

if args.verbose:
    args.columns = False

# Number of test iterations to average for timing.
test_iterations = args.iterations

scheme_preload = [  
                    "opencog/atomspace/core_types.scm",
                    "opencog/scm/utilities.scm" 
                 ]

def loop_time_per_op():
    start = time.time()
    n = 1000000
    for i in xrange(n):
        pass
    finish = time.time()
    return ((finish - start) / n)


# Preps - Perform any untimed one-time setup required for a test.

def prep_none(atomspace):
    pass

def prep_traverse_10K(atomspace):
    """Add n nodes in atomspace with python bindings"""
    n = 10000
    for i in xrange(n):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n, atomspace.get_atoms_by_type(types.ConceptNode)

def prep_traverse_100K(atomspace):
    """Add n nodes in atomspace with python bindings"""
    n = 100000
    for i in xrange(n):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n, atomspace.get_atoms_by_type(types.ConceptNode)

def prep_traverse_1M(atomspace):
    """Add n nodes in atomspace with python bindings"""
    n = 1000000
    for i in xrange(n):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n, atomspace.get_atoms_by_type(types.ConceptNode)

def prep_scheme(atomspace):
    scheme.__init__(atomspace)

def prep_bind(atomspace):
    scheme.__init__(atomspace)
    for scheme_file in scheme_preload:
        load_scm(atomspace, scheme_file)

    # Define several animals and something of a different type as well
    scheme_animals = \
        '''
        (InheritanceLink (ConceptNode "Frog") (ConceptNode "animal"))
        (InheritanceLink (ConceptNode "Zebra") (ConceptNode "animal"))
        (InheritanceLink (ConceptNode "Deer") (ConceptNode "animal"))
        (InheritanceLink (ConceptNode "Spaceship") (ConceptNode "machine"))
        '''
    scheme_eval_h(atomspace, scheme_animals)

def prep_bind_python(atomspace):
    prep_bind(atomspace)

    # Define a graph search query
    bind_link_query = \
        '''
        (BindLink
            ;; The variable to be bound
            (VariableNode "$var")
            (ImplicationLink
                ;; The pattern to be searched for
                (InheritanceLink
                    (VariableNode "$var")
                    (ConceptNode "animal")
                )
                ;; The value to be returned.
                (VariableNode "$var")
            )
        )
        '''
    return scheme_eval_h(atomspace, bind_link_query)

def prep_bind_scheme(atomspace):
    prep_bind(atomspace)

   # Define a graph search query
    scheme_query = \
        '''
        (define find-animals
          (BindLink
            ;; The variable to be bound
            (VariableNode "$var")
            (ImplicationLink
              ;; The pattern to be searched for
              (InheritanceLink
                 (VariableNode "$var")
                 (ConceptNode "animal")
              )

              ;; The value to be returned.
              (VariableNode "$var")
            )
          )
        )
        '''
    scheme_eval_h(atomspace, scheme_query)


# Tests

def test_add_nodes(atomspace, prep_handle):
    """Add n nodes in atomspace with python bindings"""
    n = 100000
    for i in xrange(n):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n

def test_add_nodes_large(atomspace, prep_handle):
    """Add n nodes in atomspace with python bindings"""
    n = 500000
    for i in xrange(n):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
    return n

def test_add_connected(atomspace, prep_handle):
    """Add n nodes and create a complete (fully-connected) graph in atomspace
    and returns the number of items processed
    """
    n = 1000
    offset = atomspace.add_node(types.ConceptNode, "Starting handle offset")
    offset = offset.h.value()

    for i in xrange(1, n+1):
        atomspace.add_node(types.ConceptNode, str(i), TruthValue(.5, .5))
        for j in xrange(1, i):
            atomspace.add_link(types.HebbianLink,
                       [Handle(i + offset), Handle(j + offset)],
                       TruthValue(.2, .3))

    # Number of vertices plus number of edges in a fully connected graph
    return n + (n**2 - n) / 2


# Traversal tests
def test_bare_traversal(atomspace, prep_traverse_result):
    atom_count, atom_list = prep_traverse_result
    counter = 0;
    for atom in atom_list:
        counter += 1

    # Prep for traversal returned the node count. The above iterates all.
    return atom_count

def test_resolve_traversal(atomspace, prep_traverse_result):
    atom_count, atom_list = prep_traverse_result
    for atom in atom_list:
        if not atom:
            print "test_resolve_traversal - atom didn't resolve"
            break

    # Prep for traversal returned the node count. The above iterates all.
    return atom_count


# Bindlink tests

def test_stub_bindlink(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = stub_bindlink(atomspace, prep_handle)
    return n

def test_bind(atomspace, prep_handle):
    n = 10000
    for i in xrange(n):
        result = bindlink(atomspace, prep_handle)
    return n

def test_validate_bindlink(atomspace, prep_handle):
    n = 10000
    for i in xrange(n):
        result = validate_bindlink(atomspace, prep_handle)
    return n


# Scheme tests

def test_scheme_eval(atomspace, prep_handle):
    n = 10000
    for i in xrange(n):
        result = scheme_eval_h(atomspace, '(+ 2 2)')
    return n

def test_bind_scheme(atomspace, prep_handle):
    n = 10000
    for i in xrange(n):
        result = scheme_eval_h(atomspace, '(cog-bind find-animals)')
    return n

def test_add_nodes_scheme(atomspace, prep_handle):
    """Add n nodes in atomspace using scheme"""
    n = 10000
    for i in xrange(n):
        scheme = 'cog-new-node \'ConceptNode "' + str(i) + \
                '" (cog-new-stv 0.5 0.5)'
        scheme_eval_h(atomspace, scheme)
    return n

def test_add_nodes_sugar(atomspace, prep_handle):
    """Add n nodes in atomspace using scheme with syntactic sugar"""
    n = 10000
    for i in xrange(n):
        scheme = '\'ConceptNode "' + str(i) + '" (cog-new-stv 0.5 0.5)'
        scheme_eval_h(atomspace, scheme)
    return n

# Run the prep, then the test.
# Note: the AtomSpace gets deleted when this function completes and
# atomspace goes out of scope. So each test begins with a new AtomSpace.
def do_test(prep, test, description, op_time_adjustment):
    """Runs tests and prints the test name and performance"""
    if not args.columns: 
        print description
    if (test != None):
        total_time = 0
        total_ops = 0
        for i in xrange(test_iterations):
            # Prep the test
            atomspace = AtomSpace()
            prep_result = prep(atomspace)

            # Time the test
            start = time.time()
            ops = test(atomspace, prep_result)
            finish = time.time()

            # Add the time and ops for this iteration
            total_time += finish - start
            total_ops += ops

        average_time = total_time / test_iterations
        average_ops = total_ops / test_iterations
        adjusted_time = average_time - (ops * op_time_adjustment)

        # Print timing results
        if args.verbose:
            time_string = "{0:.03f}".format(adjusted_time)
            print "  Total:   {0: >12}s".format(time_string)

            test_ops = "{0:,d}".format(int(average_ops))
            print "  Ops:     {0: >12}".format(test_ops)

        op_time = "{0:.03f}".format(adjusted_time * 1000000 / ops)
        ops_sec = "{0:,d}".format(int(ops / adjusted_time))
        if args.columns:
            print "{0: <40} {1: >12}µs {2: >15}".format(description, op_time,
                                                        ops_sec)
        else:
            print "  Op time: {0: >12}µs".format(op_time)
            print "  Ops/sec: {0: >12}".format(ops_sec)
            print

# The preps column contains functions that setup each test that are not timed.
# The result returned by the prep is passed through to the test so you can
# build complex predicates etc. without affecting test times.
# Prep function     # Test function         # Test description
tests = [
(['all'],                   None,               None,                   "-- Testing Node Adds --"),
(['node','spread'],         prep_none,          test_add_nodes,         "Add nodes - Cython"),
(['node'],                  prep_none,          test_add_nodes_large,   "Add nodes - Cython (500K)"),
(['node'],                  prep_none,          test_add_connected,     "Add fully connected nodes"),

(['all'],                   None,               None,                   "-- Testing Atom Traversal --"),
(['traverse'],              prep_traverse_100K, test_bare_traversal,    "Bare atom traversal 100K - by type"),
(['traverse'],              prep_traverse_10K,  test_resolve_traversal, "Resolve handle 10K - by type"),
(['traverse','spread'],     prep_traverse_100K, test_resolve_traversal, "Resolve handle 100K - by type"),
(['traverse'],              prep_traverse_1M,   test_resolve_traversal, "Resolve handle 1M - by type"),

(['all'],                   None,               None,                   "-- Testing Bind --"),
(['bindlink'],              prep_none,          test_stub_bindlink,     "Bind stub_bindlink - Cython"),
(['bindlink','spread'],     prep_bind_python,   test_bind,              "Bind bindlink - Cython"),
(['bindlink'],              prep_bind_python,   test_validate_bindlink, "Bind validate_bindlink - Cython"),

(['all'],                   None,               None,                   "-- Testing Scheme Eval --"),
(['scheme','spread'],       prep_scheme,        test_scheme_eval,       "Test scheme_eval_h(+ 2 2)"),
(['scheme'],                prep_bind_scheme,   test_bind_scheme,       "Bind - cog-bind"),
(['scheme'],                prep_scheme,        test_add_nodes_scheme,  "Add nodes - Scheme cog-new-node"),
(['scheme'],                prep_scheme,        test_add_nodes_sugar,   "Add nodes - Scheme ConceptNode sugar"),
]


print 
print "--- OpenCog Python Benchmark - ", str(datetime.datetime.now()), "---"
print

# Get the per-op adjustment to account for the time of the loops themselves.
op_time_adjustment = loop_time_per_op()
if args.verbose or args.iterations != 10:
    if args.iterations > 1:
        print "Test times averaged over {0:d} iterations.".format(test_iterations)
    else:
        print "Test times averaged over 1 iteration."

if not args.verbose:
    print

if args.verbose:
    print "Per-op loop adjustment: {0:.03f}µs.".format(op_time_adjustment * 1000000)
    print

if args.columns:
    print "{0: <40} {1: >14} {2: >15}".format("Test", "Time per op", "Ops per second")
    print "{0: <40} {1: >14} {2: >15}".format("----", "-----------", "--------------")

# Run all the tests.
for run_list, prep, test, description in tests:
    if args.all or args.test in run_list:
        do_test(prep, test, description, op_time_adjustment)

# Print an extra line to offset the benchmarks
print
