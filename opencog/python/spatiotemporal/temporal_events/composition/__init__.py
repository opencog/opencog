from math import fabs, sqrt
from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaConvolution

__author__ = 'keyvan'
UNIFORM_REFERENCE = uniform(0, 1)
relation_formula = RelationFormulaConvolution()


def unpack_partial(relation, start_reference=0, length_reference=1):
    before, same, after = relation
    similarity = same / (1 - fabs(before - after))

    solutions = []

    lengths = length_reference / similarity**2, length_reference * similarity**2
    for i in xrange(2):
        length_solution = lengths[i]
        starts = [
            start_reference - (length_solution*before - 0.5*length_reference),
            start_reference + length_reference - (length_reference*before + 0.5*length_solution)
        ]
        start_solution = starts[i]

        comparison_operand = 0.5 * (length_reference / length_solution)**((-1)**i)
        if before <= comparison_operand:
            start_solution = start_reference + length_reference - sqrt(2*before*length_solution*length_reference)
        elif before > 1 - comparison_operand:
            start_solution = start_reference - length_solution + sqrt(2*after*length_solution*length_reference)

        if i == 0 or i == 1 and solutions[0].args != (start_solution, length_solution):
            solutions.append((round(start_solution, 15), round(length_solution, 15)))

    return solutions


def unpack(relation_a_b_beginning, relation_a_b_ending):
    pass


def combine(relation_a_b_beginning, relation_a_b_ending, relation_b_beginning_c, relation_b_ending_c):
    pass


def compose(relation_a_b, relation_b_c):
    before_a_b, same_a_b, after_a_b = relation_a_b
    before_b_c, same_b_c, after_b_c = relation_b_c

    if same_a_b == 1:
        return {relation_b_c}

    if same_b_c == 1:
        return {relation_a_b}

    if before_a_b + before_b_c == 2.0:
        return {(1.0, 0.0, 0.0)}

    if after_a_b + after_b_c == 2.0:
        return {(0.0, 0.0, 1.0)}

    if before_a_b + after_b_c == 2.0 or after_a_b + before_b_c == 2.0:
        return {}

    candidates_a = unpack_partial(relation_a_b)
    candidates_c = unpack_partial(tuple(reversed(relation_b_c)))

    return {relation_formula.compare(a, c) for a in candidates_a for c in candidates_c}

if __name__ == '__main__':
    a = uniform(-1.5, 2)
    c = uniform(0.2, 3)
    print relation_formula.compare(a, c)
    print compose(relation_formula.compare(a, UNIFORM_REFERENCE), relation_formula.compare(UNIFORM_REFERENCE, c))

    # relation = relation_formula.compare(a, UNIFORM_REFERENCE)
    # reversed_relation = tuple(reversed(relation_formula.compare(UNIFORM_REFERENCE, a)))
    # print relation
    # for i in [(relation_formula.compare(s, UNIFORM_REFERENCE), s.args) for s in unpack_partial(relation)]:
    #     print i
    #
    # print reversed_relation
    # for i in [(relation_formula.compare(s, UNIFORM_REFERENCE), s.args) for s in unpack_partial(reversed_relation)]:
    #     print i
