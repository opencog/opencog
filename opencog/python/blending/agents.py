__author__ = 'Keyvan'

from opencog.atomspace import AtomSpace, Node, find_links_upward
from opencog.cogserver import MindAgent
from random import random as rand

class DummyBlendingAgent(MindAgent):

    def run(self, atomspace):
        nodes = []
        for node in atomspace.get_atoms_by_type("ConceptNode"):
            nodes.append(node)
        if len(nodes) <= 1:
            return

        first_node_index = rand(0,len(nodes))
        second_node_index = rand(0,len(nodes))
        while second_node_index == first_node_index:
            second_node_index = rand(0,len(nodes))
        C1 = nodes[first_node_index]
        C2 = nodes[second_node_index]

        C1_links, C2_links = [], []
        for link in find_links_upward(C1):
            if link.type_name == "SimilarityLink":
                C1_links.append(link)
        for link in find_links_upward(C2):
            if link.type_name == "SimilarityLink":
                C2_links.append(link)

        C3 = atomspace.add_node("ConceptNode", C1.name + C2.name)
        links = C1_links + C2_links
        for i in range(len(links)):
            link = links[rand(0,len(links))]
            for node in link.out:
                atomspace.add_link("SimilarityLink", node)






