"""
Python-formatted data definitions for smokes_example.py

The Scheme-formatted definitions are in this file:
  https://github.com/opencog/test-datasets/blob/master/pln/tuffy/smokes/smokes.scm
They are also defined here in Python format for testing.
"""

__author__ = 'Cosmo Harrigan'

from opencog.atomspace import AtomSpace, TruthValue, types

atomspace = AtomSpace()

# Basic variable definitions
X = atomspace.add_node(types.VariableNode, "$X")
lX = atomspace.add_link(types.ListLink, [X])

Y = atomspace.add_node(types.VariableNode, "$Y")
lY = atomspace.add_link(types.ListLink, [Y])

X_Y = atomspace.add_link(types.ListLink, [X, Y])

full_confidence = TruthValue().confidence_to_count(1)
crisp_true = TruthValue(1, full_confidence)

# Anna smokes.
Anna = atomspace.add_node(types.ConceptNode, "Anna")
smokes = atomspace.add_node(types.PredicateNode, "smokes")
atomspace.add_link(types.EvaluationLink,
                   [smokes, atomspace.add_link(types.ListLink, [Anna])],
                   crisp_true)

# Edward smokes.
Edward = atomspace.add_node(types.ConceptNode, "Edward")
atomspace.add_link(types.EvaluationLink,
                   [smokes, atomspace.add_link(types.ListLink, [Edward])],
                   crisp_true)

# If X smokes, then X has cancer.
cancer = atomspace.add_node(types.PredicateNode, "cancer")
smokes_X = atomspace.add_link(types.EvaluationLink, [smokes, lX])
cancer_X = atomspace.add_link(types.EvaluationLink, [cancer, lX])

atomspace.add_link(types.ImplicationLink,
                   [smokes_X, cancer_X],
                   TruthValue(0.6225, full_confidence))

# In the case that X and Y are friends, if X smokes then so does Y.
friends = atomspace.add_node(types.PredicateNode, "friends")
friends_X_Y = atomspace.add_link(types.EvaluationLink, [friends, X_Y])

smokes_Y = atomspace.add_link(types.EvaluationLink, [smokes, lY])

implication_smokes_X_smokes_Y = atomspace.add_link(
    types.ImplicationLink, [smokes_X, smokes_Y])

atomspace.add_link(types.ImplicationLink,
                   [friends_X_Y, implication_smokes_X_smokes_Y],
                   TruthValue(0.5987, full_confidence))

# Anna is friends with Bob.
Bob = atomspace.add_node(types.ConceptNode, "Bob")
atomspace.add_link(
    types.EvaluationLink,
    [friends, atomspace.add_link(types.ListLink, [Anna, Bob])], crisp_true)

# Anna is friends with Bob.
atomspace.add_link(
    types.EvaluationLink,
    [friends, atomspace.add_link(types.ListLink, [Bob, Anna])], crisp_true)

# Edward is friends with Frank.
Frank = atomspace.add_node(types.ConceptNode, "Frank")
atomspace.add_link(
    types.EvaluationLink,
    [friends, atomspace.add_link(types.ListLink, [Edward, Frank])], crisp_true)
