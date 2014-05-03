from copy import deepcopy
import numpy
import random
from spatiotemporal.temporal_events import RelationFormulaConvolution
from spatiotemporal.temporal_events.composition import unpack
from spatiotemporal.temporal_events.composition.emperical_distribution import overlaps
from spatiotemporal.temporal_events.composition.rail_framework import RailwaySystem
from spatiotemporal.temporal_events.trapezium import generate_random_events, TemporalEventTrapezium


class Node(object):
    def __init__(self, rails, unpack_states):
        self.rails = rails
        self.unpack_states = unpack_states


def is_solution(node):
    for state in node.unpack_states.values():
        if not state:
            return False
    return True


def expand(node, relations):
    successors = []
    rails = node.rails
    new_unpack_states = deepcopy(node.unpack_states)
    relation = None
    for key, unpacked in node.unpack_states.items():
        if not unpacked:
            relation = key
            new_unpack_states[relation] = True
            break

    if relation is None:
        return []

    temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2 = relation
    relation = relations[relation]
    start_reference = rails[temporal_event_2_key][portion_index_2].a
    length_reference = rails[temporal_event_2_key][portion_index_2].length

    if relation == (1.0, 0.0, 0.0):
        new_rails = deepcopy(rails)
        new_rails.bind_wagons_before_horizontal(temporal_event_1_key, portion_index_1, temporal_event_2_key,
                                                portion_index_2)
        return [Node(new_rails, new_unpack_states)]

    if relation == (0.0, 0.0, 1.0):
        new_rails = deepcopy(rails)
        new_rails.bind_wagons_after_horizontal(temporal_event_1_key, portion_index_1, temporal_event_2_key,
                                               portion_index_2)
        return [Node(new_rails, new_unpack_states)]

    candidates = unpack(relation, start_reference, length_reference)
    for a, b in candidates:
        reference = rails[temporal_event_2_key][portion_index_2]

        new_rails = deepcopy(rails)
        new_rails.move_and_bind_vertical(temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2,
                                         a, b)
        new_reference = new_rails[temporal_event_2_key][portion_index_2]
        if new_reference.a != reference.a or new_reference.b != reference.b:
            continue

        successors.append(Node(new_rails, deepcopy(new_unpack_states)))

    return successors


class DepthFirstSearchComposition(object):
    def __init__(self):
        self.stack = []
        self.relations = {}
        self.events = set()

    def add_relation(self, temporal_event_1_key, portion_index_1,
                     temporal_event_2_key, portion_index_2, relation):
        self.events.add(temporal_event_1_key)
        self.events.add(temporal_event_2_key)
        self.relations[temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2] = relation

    def find_solutions(self, initial_system):
        solutions = []
        initial_unpack_states = {}
        for key in self.relations:
            initial_unpack_states[key] = False
        start = Node(initial_system, initial_unpack_states)

        stack = [start]
        while stack:
            node = stack.pop()
            if is_solution(node):
                solutions.append(node.rails)
            stack += expand(node, self.relations)

        return solutions


def convert_rail_to_trapezium_event(railway_system, rail_key):
    a = railway_system[rail_key][0].a
    beginning = railway_system[rail_key][0].b
    ending = railway_system[rail_key][1].a
    b = railway_system[rail_key][1].b

    return TemporalEventTrapezium(a, b, beginning, ending)


if __name__ == '__main__':
    search_tree = DepthFirstSearchComposition()
    formula = RelationFormulaConvolution()
    A, B, C = generate_random_events(3)
    for event in [A, B, C]:
        p = ''
        for point in [event.a, event.beginning, event.ending, event.b]:
            p += str((point - A.a) / (A.beginning - A.a)) + ', '
        print p

    # A = TemporalEventTrapezium(0, 30, 10, 20)
    # B = TemporalEventTrapezium(8, 22, 15, 16)
    # C = TemporalEventTrapezium(0, 30, 10, 20)
    goal = []
    events = {'A': A, 'B': B, 'C': C}
    for a_key, b_key in [('A', 'B'), ('B', 'C')]:
        a, b = events[a_key], events[b_key]
        for portion_index_a in [0, 1]:
            for portion_index_b in [0, 1]:
                relation = formula.compare(a[portion_index_a], b[portion_index_b])
                goal.append(relation)
                search_tree.add_relation(a_key, portion_index_a, b_key, portion_index_b, relation)

    goal = numpy.array(goal)

    rails = RailwaySystem()
    for event_key in events:
        rails.add_rail(event_key)

    solutions = search_tree.find_solutions(rails)

    for railway_system in solutions:
        estimate = []
        A = convert_rail_to_trapezium_event(railway_system, 'A')
        B = convert_rail_to_trapezium_event(railway_system, 'B')
        C = convert_rail_to_trapezium_event(railway_system, 'C')
        events = {'A': A, 'B': B, 'C': C}
        for a_key, b_key in [('A', 'B'), ('B', 'C')]:
            a, b = events[a_key], events[b_key]
            for portion_index_a in [0, 1]:
                for portion_index_b in [0, 1]:
                    relation = formula.compare(a[portion_index_a], b[portion_index_b])
                    estimate.append(relation)
        print goal - numpy.array(estimate)


"""
0.0, 1.0, 3.95895473838, 4.49707162177,
-0.612107375715, 4.16960183244, 4.45574646245, 16.9067080822,
4.65660415787, 6.70232672098, 17.3980110842, 17.8023626962,

0.0, 1.0, 43.9490138429, 58.2172787905,
216.760424476, 276.752908482, 296.416558036, 387.454518564,
93.3980514706, 309.730217663, 387.304504184, 477.604687366,

0.0, 1.0, 1.18958954041, 2.54221796041,
1.86934798288, 1.90946709733, 2.09485603768, 2.13348835072,
-0.531899401165, -0.262333114979, 2.77404081223, 3.17684405284,
"""
