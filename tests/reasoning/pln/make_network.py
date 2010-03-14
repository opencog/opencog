# A simple Python script to make a random AtomSpace of any size. There are two parameters, which are tacky constants.
# The output of this script should be redirected to make a new Scheme file for the CogServer to load.

num_nodes = 100
num_links_per_node = 10

from random import randint

def node(n):
    return '(ConceptNode "x'+str(n)+'" (stv 1.0 0.999))'

def link(s1,s2):
    return '(SimilarityLink (stv 1.0 0.999) '+str(s1)+' '+str(s2)+')'

for i in xrange(0, num_nodes):
    print node(i)

for i in xrange(0, num_nodes*num_links_per_node):
    s1 = node(randint(0,num_nodes))
    s2 = node(randint(0,num_nodes))
    if (s1 != s2): # No self-links. That wouldn't make sense, and would break stuff.
        print link(s1, s2)

