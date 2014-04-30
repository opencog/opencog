from spatiotemporal.temporal_events.composition import unpack


class Node(object):
    def __init__(self, relation, start_reference=0, length_reference=1):
        self.relation = relation
        self.start_reference = start_reference
        self.length_reference = length_reference


def expand(node):
    return unpack(node.relation, node.start_reference, node.length_reference)


class DepthFirstSearchComposition(object):
    def __init__(self):
        self.stack = []
