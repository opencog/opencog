"""
Converts MLN evidence.db files into AtomSpace representation

Accepts lines that match the following formats:

predicate(argument1, argument2, ..., argumentN)
!predicate(argument1, argument2, ..., argumentN)

Output format is:

(EvaluationLink (stv 1 1)
    (PredicateNode "predicate")
    (ListLink
        (ConceptNode "argument1")
        (ConceptNode "argument2")
          ...
        (ConceptNode "argumentN")))

or

(NotLink (stv 1 1)
    (EvaluationLink
        (PredicateNode "predicate")
        (ListLink
            (ConceptNode "argument1")
            (ConceptNode "argument2")
              ...
            (ConceptNode "argumentN"))))

Todo:
  - Should it set node probabilities?
  - Confirm how negation should be handled
"""

from opencog.atomspace import AtomSpace, types, TruthValue

__author__ = 'Cosmo Harrigan'

input = raw_input('Input filename: ')
output = raw_input('Output filename: ')

atomspace = AtomSpace()
crisp_true = TruthValue(1, TruthValue().confidence_to_count(1))

try:
    with open(input) as evidence:
        with open(output, 'w') as output:
            for line in evidence:
                if len(line) > 1:
                    # Parse: Negation
                    line = line.split('!')
                    if len(line) > 1:
                        negated = True
                        line = line[1]
                    else:
                        negated = False
                        line = line[0]

                    # Parse: Predicate
                    concepts = []
                    line = line.split('(')
                    predicate = line[0]

                    # Parse: Concepts
                    done = False
                    line = line[1].split(',')
                    for elem in line:
                        elem = elem.split(')')
                        concepts.append(elem[0])

                    # Construct OpenCog atoms

                    predicate_node = atomspace.add_node(
                        types.PredicateNode, predicate)

                    # ConceptNodes
                    concept_nodes = []
                    for concept in concepts:
                        concept_nodes.append(atomspace.add_node(
                            types.ConceptNode, concept))

                    # ListLink
                    list_link = atomspace.add_link(
                        types.ListLink, concept_nodes)

                    # EvaluationLink
                    eval_link = atomspace.add_link(
                        types.EvaluationLink, [predicate_node, list_link])
                    new_link = eval_link

                    # Determine whether the predicate is negated, and create
                    # a NotLink if necessary.
                    if negated:
                        # NotLink
                        not_link = atomspace.add_link(types.NotLink, [eval_link])

                        # TruthValue
                        atomspace.set_tv(not_link.h, crisp_true)

                        new_link = not_link
                    else:
                        # TruthValue
                        atomspace.set_tv(eval_link.h, crisp_true)

                    output.write(new_link.__str__())

except (OSError, IOError), e:
    print('Exception: {0}'.format(e))
