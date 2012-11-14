__author__ = 'Keyvan'

from opencog.atomspace import AtomSpace, Node, find_links_upward
from opencog.cogserver import MindAgent
from random import randrange

class DummyBlendingAgent(MindAgent):

    def run(self, atomspace):
        # Define a set to add the source ConceptNodes to which we will retrieve from the AtomSpace
        nodes = []

        # Request all nodes of type 'ConceptNode' from the AtomSpace and loop through them
        for node in atomspace.get_atoms_by_type('ConceptNode'):
            # Append each ConceptNode to our own nodes set

            nodes.append(node)
        # Check to make sure we actually got back some nodes of the type we want (ConceptNode)
        if len(nodes) <= 1:
            # Otherwise we might as well stop here
            return

        # Randomly select a number (limited by the size of our node set) for the first node
        first_node_index = randrange(0,len(nodes))
        # Randomly select a number for the second node with the same limitations
        second_node_index = randrange(0,len(nodes))
        # Check to see we didn't unfortunately get the same number twice
        while second_node_index == first_node_index:
            # If we did, pick a new second number
            second_node_index = randrange(0,len(nodes))

        # Retrieve the first blending node by its index and keep it in C1
        C1 = nodes[first_node_index]
        # Retrieve the second blending node by its index and keep it in C2
        C2 = nodes[second_node_index]

        # Prepare two lists to store the links of C1 and C2 in
        C1_links, C2_links = [], []

        # Loop through the links in C1
        for link in find_links_upward(C1):
            # Check to see if it's a SimilarityLink
            if link.type_name == 'SimilarityLink':
                # If so, add it to C1_links
                C1_links.append(link)
        # Loop through the links in C2
        for link in find_links_upward(C2):
            # Check to see if it's a SimilarityLink
            if link.type_name == 'SimilarityLink':
                # If so, add it to C2_links
                C2_links.append(link)

        
        C3 = atomspace.add_node('ConceptNode', C1.name + C2.name)
        links = C1_links + C2_links
        for i in range(len(links)):
            link = links[randrange(0,len(links))]
            for node in link.out:
                link =atomspace.add_link('SimilarityLink', (C3,node))
                print link






