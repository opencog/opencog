from math import fabs, sqrt
from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaConvolution

__author__ = 'keyvan'
UNIFORM_REFERENCE = uniform(0, 1)


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
            solutions.append(uniform(start_solution, length_solution))

    return solutions


def compose(relation_a_b, relation_b_c):
    pass


if __name__ == '__main__':
    f = RelationFormulaConvolution()
    a = uniform(0.1, 0.8)
    relation = f.compare(a, UNIFORM_REFERENCE)
    print relation
    for i in [(f.compare(s, UNIFORM_REFERENCE), s.args) for s in unpack_partial(relation)]:
        print i

