__author__ = 'keyvan'

from rdflib import Graph
from opencog.atomspace import AtomSpace, TruthValue

DEFAULT_TV = TruthValue(1,1)

def map_from_path(rdf_path, atomspace):
    graph = Graph()
    graph.load(rdf_path)

    for s, p, o in graph:
        subject, predicate, object = unicode(s).encode('ascii','replace'),\
                                     unicode(p).encode('ascii','replace'),\
                                     unicode(o).encode('ascii','replace')
        try:
            object_value = int(object)
            object_type = 'NumberNode'
        except:
            object_type = 'ConceptNode'

        atomspace.add_link('EvaluationLink',
            [atomspace.add_node('PredicateLink', predicate, DEFAULT_TV),
             atomspace.add_link('ListLink',
                [atomspace.add_node('ObjectEntityNode', subject, DEFAULT_TV),
                atomspace.add_node(object_type, object, DEFAULT_TV)
                ])
             ])

if __name__ == '__main__':
    atomspace = AtomSpace()
    map_from_path('http://rdf.freebase.com/rdf/en.hewlett-packard', atomspace)
    atomspace.print_list()

