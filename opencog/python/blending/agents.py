__author__ = 'Keyvan'

from opencog.atomspace import AtomSpace, Node, find_links_upward
from opencog.cogserver import MindAgent
from random import randrange

class DummyBlendingAgent(MindAgent):

    def run(self, atomspace):
        nodes = []
        for node in atomspace.get_atoms_by_type('ConceptNode'):
            nodes.append(node)
        if len(nodes) <= 1:
            return

        first_node_index = randrange(0,len(nodes))
        second_node_index = randrange(0,len(nodes))
        while second_node_index == first_node_index:
            second_node_index = randrange(0,len(nodes))
        C1 = nodes[first_node_index]
        C2 = nodes[second_node_index]

        C1_links, C2_links = [], []
        for link in find_links_upward(C1):
            if link.type_name == 'SimilarityLink':
                C1_links.append(link)
        for link in find_links_upward(C2):

            if link.type_name == 'SimilarityLink':
                C2_links.append(link)

        C3 = atomspace.add_node('ConceptNode', C1.name + '_' + C2.name)
        print C3
        links = C1_links + C2_links
        for i in range(len(links)):
            link = links[randrange(0,len(links))]
            for node in link.out:
                link =atomspace.add_link('SimilarityLink', (C3,node))
                print link






