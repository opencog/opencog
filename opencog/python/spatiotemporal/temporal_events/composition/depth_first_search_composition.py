from copy import deepcopy
import random
from spatiotemporal.temporal_events import RelationFormulaConvolution
from spatiotemporal.temporal_events.composition import unpack
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
        new_rails = deepcopy(rails)
        new_rails.move_and_bind_vertical(temporal_event_1_key, portion_index_1, temporal_event_2_key, portion_index_2,
                                         a, b)
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


if __name__ == '__main__':
    search_tree = DepthFirstSearchComposition()
    formula = RelationFormulaConvolution()
    # A, B, C = generate_random_events(3)
    # for event in [A, B, C]:
    #     p = ''
    #     for point in [event.a, event.beginning, event.ending, event.b]:
    #         p += str((point - A.a) / (A.beginning - A.a)) + ', '
    #     print p

    A = TemporalEventTrapezium(0.0, 3.76285794052, 1.0, 2.36411109829)
    B = TemporalEventTrapezium(0.429332220194, 0.618402145058, 0.459203813555, 0.565328983453)
    C = TemporalEventTrapezium(0.124194304788, 4.60495737242, 1.16915491876, 3.62004160264)

    events = {'A': A, 'B': B, 'C': C}
    for a_key, b_key in [('A', 'B'), ('B', 'C')]:
        a, b = events[a_key], events[b_key]
        for portion_index_a in [0, 1]:
            for portion_index_b in [0, 1]:
                search_tree.add_relation(a_key, portion_index_a, b_key, portion_index_b,
                                         formula.compare(a[portion_index_a], b[portion_index_b]))

    rails = RailwaySystem()
    for event_key in events:
        rails.add_rail(event_key)

    print search_tree.find_solutions(rails)