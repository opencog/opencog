from math import fabs, sqrt
from non_linear_least_squares import DecompositionFitter
from spatiotemporal.temporal_events import FormulaCreator
from depth_first_search_composition import DepthFirstSearchComposition, convert_rail_to_trapezium_event
from spatiotemporal.temporal_events.composition.railway_framework import RailwaySystem
from spatiotemporal.temporal_events.relation_formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import generate_random_events
from spatiotemporal.temporal_events.util import compute_railway_strength


__author__ = 'keyvan'


def compose(relation_a_b, relation_b_c):
    strengths_by_solution = {}
    formula_a_b = DecompositionFitter(relation_a_b)
    formula_b_c = DecompositionFitter(relation_b_c)

    search_tree = DepthFirstSearchComposition()
    dist_key_by_index = {0: 'beginning', 1: 'ending'}
    for temporal_event_1_key, temporal_event_2_key, formula in [('A', 'B', formula_a_b), ('B', 'C', formula_b_c)]:
        for portion_index_1 in [0, 1]:
            for portion_index_2 in [0, 1]:
                search_tree.add_relation(temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2,
                                         formula.compare(dist_key_by_index[portion_index_1],
                                                             dist_key_by_index[portion_index_2]))
    rails = RailwaySystem()
    rails.add_rail('A')
    rails.add_rail('B')
    rails.add_rail('C')

    solutions = search_tree.find_solutions(rails)
    strength_total, strength_per_solution = compute_railway_strength(solutions, goals=[('A', 'C')])
    print 'truth value -- total: {0}, each: {1}'.format(strength_total, strength_per_solution)
    if strength_total == 0:
        return None, 0
    for railway_system in solutions:
        railway_system.compress()
        A = convert_rail_to_trapezium_event(railway_system, 'A')
        B = convert_rail_to_trapezium_event(railway_system, 'B')
        C = convert_rail_to_trapezium_event(railway_system, 'C')
        solution = tuple((A * C).to_list())
        if solution in strengths_by_solution:
            strengths_by_solution[solution] += strength_per_solution
        else:
            strengths_by_solution[solution] = strength_per_solution

    result = []
    for solution in strengths_by_solution:
        strength = strengths_by_solution[solution]
        print solution, strength
        result.append((TemporalRelation.from_list(solution), strength))

    return result, strength_total



if __name__ == '__main__':
    A, B, C = generate_random_events(3)
    print 'actual', (A * C).to_list()
    print
    compose(A * B, B * C)
