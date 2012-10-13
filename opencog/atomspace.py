__author__ = 'Keyvan'

marker = object()

types = marker
types.TimeNode = marker
types.ConceptNode = marker
types.InheritanceLink = marker
types.ListLink = marker
types.AtTimeLink = marker

class TruthValue(object):
    def __init__(self, number_of_evidences, confidence):
        self.number_of_evidences = number_of_evidences
        self.confidence = confidence

DEFAULT_TRUTH_VALUE = TruthValue(1,1)

class Node(object):
    def __init__(self, type, string):
        self.type = type
        self.string = string

    def __eq__(self, other):
        return self.string == other.string

class Link(list):
    def __init__(self, link_type, list_of_nodes, truth_value=DEFAULT_TRUTH_VALUE):
        self.type = link_type
        self += list_of_nodes
        self.truth_value = truth_value

class AtomSpace(object):

    def __init__(self):
        self.nodes = set()
        self.links = set()

    def add_node(self, type, string):
        self.nodes.add(Node(type,string))

    def add_link(self, link_type, list_of_nodes, truth_value=DEFAULT_TRUTH_VALUE):
        self.links.add()


class Atom(object):
    pass

