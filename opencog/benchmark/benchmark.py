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

import time, datetime
from opencog.atomspace import AtomSpace, TruthValue, Handle, types
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h
from opencog.bindlink import stub_bindlink, bindlink, validate_bindlink

__authors__ = 'Cosmo Harrigan, Curtis Faith'

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

def test_stub_bindlink(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = stub_bindlink(atomspace, prep_handle)
    return n

def test_bind(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = bindlink(atomspace, prep_handle)
    return n

def test_validate_bindlink(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = validate_bindlink(atomspace, prep_handle)
    return n

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

def test_scheme_eval(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = scheme_eval_h(atomspace, '(+ 2 2)')
    return n

def test_bind_scheme(atomspace, prep_handle):
    n = 100000
    for i in xrange(n):
        result = scheme_eval_h(atomspace, '(cog-bind find-animals)')
    return n

def test_add_nodes_scheme(atomspace, prep_handle):
    """Add n nodes in atomspace using scheme"""
    n = 100000
    for i in xrange(n):
        scheme = 'cog-new-node \'ConceptNode "' + str(i) + \
                '" (cog-new-stv 0.5 0.5)'
        scheme_eval_h(atomspace, scheme)
    return n

def test_add_nodes_sugar(atomspace, prep_handle):
    """Add n nodes in atomspace using scheme with syntactic sugar"""
    n = 100000
    for i in xrange(n):
        scheme = '\'ConceptNode "' + str(i) + '" (cog-new-stv 0.5 0.5)'
        scheme_eval_h(atomspace, scheme)
    return n

# Run the prep, then the test.
# Note: the AtomSpace gets deleted when this function completes and
# atomspace goes out of scope. So each test begins with a new AtomSpace.
def do_test(prep, test, description, op_time_adjustment):
    """Runs tests and prints the test name and performance"""
    print description
    if (test != None):
        # Prep the test
        atomspace = AtomSpace()
        prep_handle = prep(atomspace)

        # Time the test
        start = time.time()
        ops = test(atomspace, prep_handle)
        finish = time.time()
        adjusted_time = (finish - start) - (ops * op_time_adjustment)

        # Print timing results
        time_string = "{0:.03f}".format(adjusted_time)
        print "  Total:   {0: >12}s".format(time_string)

        time_string = "{0:,d}".format(ops)
        print "  Ops:     {0: >12}".format(time_string)

        time_string = "{0:.03f}".format(adjusted_time * 1000000 / ops)
        print "  Op time: {0: >12}µs".format(time_string)

        time_string = "{0:,d}".format(int(ops / adjusted_time))
        print "  Ops/sec: {0: >12}".format(time_string)
    print

# The preps column contains functions that setup each test that are not timed.
# The result returned by the prep is passed through to the test so you can
# build complex predicates etc. without affecting test times.
tests = [
# Prep function     # Test function         # Test description
(None,              None,                   "-- Testing Node Adds --"),
(prep_none,         test_add_nodes,         "Add nodes - Cython"),
(prep_none,         test_add_nodes_large,   "Add nodes - Cython (500K)"),
(prep_none,         test_add_connected,     "Add fully connected nodes"),

(None,              None,                   "-- Testing Bind --"),
(prep_none,         test_stub_bindlink,     "Bind stub_bindlink - Cython"),
(prep_bind_python,  test_bind,              "Bind bindlink - Cython"),
(prep_bind_python,  test_validate_bindlink, "Bind validate_bindlink - Cython"),

(None,              None,                   "-- Testing Scheme Eval --"),
(prep_scheme,       test_scheme_eval,       "Test scheme_eval_h(+ 2 2)"),
(prep_bind_scheme,  test_bind_scheme,       "Bind - cog-bind"),
(prep_scheme,       test_add_nodes_scheme,  "Add nodes - Scheme cog-new-node"),
(prep_scheme,       test_add_nodes_sugar,   "Add nodes - Scheme ConceptNode sugar"),
]

print 
print "--- OpenCog Python Benchmark - ", str(datetime.datetime.now()), "---"
print

# Get the per-op adjustment to account for the time of the loops themselves.
op_time_adjustment = loop_time_per_op()
print "Per-op loop adjustment: {0:.03f}µs".format(op_time_adjustment * 1000000)
print
    
# Run all the tests.
for prep, test, description in tests:
    do_test(prep, test, description, op_time_adjustment)
