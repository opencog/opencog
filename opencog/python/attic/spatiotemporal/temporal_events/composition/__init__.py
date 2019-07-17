from non_linear_least_squares import DecompositionFitter
from depth_first_search_composition import DepthFirstSearchComposition, convert_rail_to_trapezium_event
from spatiotemporal.temporal_events.composition.railway_framework import RailwaySystem
from spatiotemporal.temporal_events.relation_formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import generate_random_events
from spatiotemporal.temporal_events.util import compute_railway_strength


__author__ = 'keyvan'


def compose(relation_a_b, relation_b_c):
    strengths_by_solution = {}
    # Reverse engineer the (before, same, after) relations
    formula_a_b = DecompositionFitter(relation_a_b)
    formula_b_c = DecompositionFitter(relation_b_c)
    # formula.compare now calculates (before, same, after) for each relation

    # Initiating the DFS tree
    search_tree = DepthFirstSearchComposition()

    # DecompositionFitter uses 'beginning' and 'ending' as keys for compare method (as opposed to probability
    # distribution in an actual RelationFormula), facilitating the access by a dict:
    dist_key_by_index = {0: 'beginning', 1: 'ending'}
    for temporal_event_1_key, temporal_event_2_key, formula in [('A', 'B', formula_a_b), ('B', 'C', formula_b_c)]:
        # portion_index = 0 means beginning and portion_index = 1 means ending
        for portion_index_1 in [0, 1]:
            for portion_index_2 in [0, 1]:
                # Adding the known (before, same, aster) relations (formula.compare) to our algorithm
                search_tree.add_relation(temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2,
                                         formula.compare(dist_key_by_index[portion_index_1],
                                                             dist_key_by_index[portion_index_2]))

    # Initiating a RailwaySystem instance as the starting point of DFS
    rails = RailwaySystem()
    rails.add_rail('A')
    rails.add_rail('B')
    rails.add_rail('C')

    # Executing DFS
    solutions = search_tree.find_solutions(rails)

    # Now we need to see how reliable the solutions are
    strength_total, strength_per_solution = compute_railway_strength(solutions, goals=[('A', 'C')])

    # If the total strength of the solutions is zero, we can conclude that the inputs were not composable
    if strength_total == 0:
        return [], 0

    # If not, we return the solutions with their corresponding truth value
    # Some solution might be repeated, we only pass one instance of these solution but
    # add up their truth values for that single instance, we do this by some dict tricks
    for railway_system in solutions:
        railway_system.compress()
        A = convert_rail_to_trapezium_event(railway_system, 'A')
        C = convert_rail_to_trapezium_event(railway_system, 'C')
        solution = A * C
        if solution in strengths_by_solution:
            strengths_by_solution[solution] += strength_per_solution
        else:
            strengths_by_solution[solution] = strength_per_solution

    result = []
    for solution in strengths_by_solution:
        strength = strengths_by_solution[solution]
        result.append((solution, strength))

    return result, strength_total


if __name__ == '__main__':
    A, B, C = generate_random_events(3)
    print 'actual', (A * C).to_list()
    print
    solutions, strength_total = compose(A * B, B * C)
    print 'Truth Value Total:', strength_total
    for relation_a_c, strength in solutions:
        print relation_a_c.to_list(), 'Truth Value:', strength
