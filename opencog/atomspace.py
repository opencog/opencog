__author__ = 'Keyvan'

from utility.generic import Marker

types = Marker()
types.TimeNode = 'TimeNode'
types.ConceptNode = 'ConceptNode'
types.InheritanceLink = 'InheritanceLink'
types.ListLink = 'ListLink'
types.AtTimeLink = 'AtTimeLink'

class TruthValue(object):
    def __init__(self,confidence, number_of_evidences):
        self.confidence = confidence
        self.number_of_evidences = number_of_evidences

    def __repr__(self):
        return '<' + self.number_of_evidences + ', ' + self.confidence + '>'

DEFAULT_TRUTH_VALUE = TruthValue(1,1)

class Node(object):
    def __init__(self, type, string):
        self.type = type
        self.string = string

    def __eq__(self, other):
        return self.string == other.string

    def __repr__(self):
        return self.string

class Link(object):
    def __init__(self, link_type, list_of_nodes, truth_value=DEFAULT_TRUTH_VALUE):
        self.type = link_type
        self += list_of_nodes
        self.truth_value = truth_value

    def __repr__(self):
        return self.link_type + repr(self.list_of_nodes)

class AtomSpace(object):

    def __init__(self):
        self.nodes = set()
        self.links = set()

    def add_node(self, type, string):
        node = Node(type,string)
        self.nodes.add(node)
        return node

    def add_link(self, link_type, list_of_nodes, truth_value=DEFAULT_TRUTH_VALUE):
        link = Link(link_type, list_of_nodes, truth_value)
        self.links.add(link)
        return link


class Atom(object):
    pass

