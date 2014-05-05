from math import fabs, sqrt

__author__ = 'keyvan'


def unpack(relation, start_reference=0, length_reference=1):
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

        if i == 0 or i == 1 and solutions[0] != (start_solution, start_solution + length_solution):
            solution_a, solution_b = round(start_solution, 15), round(start_solution + length_solution, 15)
            solutions.append((solution_a, solution_b))

    return solutions

# print unpack((0.68354519254445345, 0.10696157642928092, 0.31645480745554644), 66.2096292233, 9.629403233)