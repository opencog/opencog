"""
Data definitions for smokes_example.py

The full definitions are in this Scheme file:
  https://github.com/opencog/test-datasets/blob/master/pln/tuffy/smokes/smokes.scm
They are redefined in Python format here for testing, due to this bug that
prevents importing Scheme files without a running cogserver:
  https://github.com/opencog/opencog/issues/530
"""

__author__ = 'Cosmo Harrigan'

from opencog.atomspace import AtomSpace, TruthValue, types

atomspace = AtomSpace()

# Basic variable definitions
X = atomspace.add_link(types.ListLink,
                       [atomspace.add_node(types.VariableNode, "$X")])

Y = atomspace.add_link(types.ListLink,
                       [atomspace.add_node(types.VariableNode, "$X")])

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
smokes = atomspace.add_node(types.PredicateNode, "smokes")
atomspace.add_link(types.EvaluationLink,
                   [smokes, atomspace.add_link(types.ListLink, [Edward])],
                   crisp_true)

# If X smokes, then X has cancer.
cancer = atomspace.add_node(types.PredicateNode, "cancer")
smokes_X = atomspace.add_link(types.EvaluationLink, [smokes, X])
cancer_X = atomspace.add_link(types.EvaluationLink, [cancer, X])

atomspace.add_link(types.ImplicationLink,
                   [smokes_X, cancer_X],
                   TruthValue(0.5, full_confidence))

# In the case that X and Y are friends, if X smokes then so does Y.
"""
(ImplicationLink (stv 0.4 1.0)
    (EvaluationLink (stv 1.0 0.0)
        friends
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (ImplicationLink
        (EvaluationLink (stv 1.0 0.0)
            smokes
            (ListLink
                (VariableNode "$X")))
        (EvaluationLink (stv 1.0 0.0)
            smokes
            (ListLink
                (VariableNode "$Y")))))
"""
friends = atomspace.add_node(types.PredicateNode, "friends")
friends_X_Y = atomspace.add_link(types.EvaluationLink, [friends, X_Y])

smokes_Y = atomspace.add_link(types.EvaluationLink, [smokes, Y])

implication_smokes_X_smokes_Y = atomspace.add_link(
    types.ImplicationLink, [smokes_X, smokes_Y])

atomspace.add_link(types.ImplicationLink,
                   [friends_X_Y, implication_smokes_X_smokes_Y])

# Anna is friends with Bob.
Bob = atomspace.add_node(types.ConceptNode, "Bob")
atomspace.add_link(
    types.EvaluationLink,
    [friends, atomspace.add_link(types.ListLink, [Anna, Bob])], crisp_true)
