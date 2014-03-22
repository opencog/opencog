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

# Ben gives Amen headache.(3 argument ListLink)

Ben = atomspace.add_node(types.ConceptNode, "Ben")
Amen = atomspace.add_node(types.ConceptNode, "Amen")
give = atomspace.add_node(types.PredicateNode, "gives")
headache = atomspace.add_node(types.ConceptNode, "headache")

alink1=atomspace.add_link(
    types.EvaluationLink,
    [give,
    atomspace.add_link(
            types.ListLink,
            [Ben, Amen, headache]
    )]
)

alink2=atomspace.add_link(
    types.EvaluationLink,
    [give,
    atomspace.add_link(
            types.ListLink,
            [Ben, Amen, Ben]
    )]
)

# Single argument ListLink , for testing

full_confidence = TruthValue().confidence_to_count(1)
crisp_true = TruthValue(1, full_confidence)
Anna = atomspace.add_node(types.ConceptNode, "Anna")
smokes = atomspace.add_node(types.PredicateNode, "smokes")
atomspace.add_link(types.EvaluationLink,
                   [smokes, atomspace.add_link(types.ListLink, [Anna])],
                   crisp_true)
