"""
Data definitions for evaluation_to_member_example.py

Todo: The Relex2Logic rules require modification to properly translate
this relationship.

See:
https://github.com/opencog/opencog/issues/530

Currently, it is represented the same way that Relex2Logic outputs it,
which is not going to work properly.
"""

__author__ = 'Cosmo Harrigan'

from opencog.atomspace import AtomSpace, TruthValue, types

atomspace = AtomSpace()

Socrates = atomspace.add_node(types.ConceptNode, "Socrates")
Man = atomspace.add_node(types.ConceptNode, "man")
Air = atomspace.add_node(types.ConceptNode, "air")
be = atomspace.add_node(types.PredicateNode, "be")
breathe = atomspace.add_node(types.PredicateNode, "breathe")

atomspace.add_link(
    types.EvaluationLink,
    [be,
     atomspace.add_link(
         types.ListLink,
         [Socrates, Man])],
    TruthValue(1, TruthValue().confidence_to_count(1)))

atomspace.add_link(
    types.EvaluationLink,
    [breathe,
     atomspace.add_link(
         types.ListLink, [Man, Air])],
    TruthValue(1, TruthValue().confidence_to_count(1)))
