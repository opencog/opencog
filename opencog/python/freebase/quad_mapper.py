__author__ = 'keyvan&ramin'

from opencog.atomspace import AtomSpace, TruthValue
import quad_reader

DEFAULT_TV = TruthValue(1,1)

def map_from_path(quad_dump_path, atomspace):
    quads = quad_reader.extract_quads(quad_dump_path)
    for quad in quads:
        try:
            value = int(quad.value)
            value_node_type = 'NumberNode'
        except:
            value_node_type = 'ConceptNode'

        if quad.value is None:
            atomspace.add_link('EvaluationLink',
                [atomspace.add_node('PredicateNode', quad.predicate, DEFAULT_TV),
                 atomspace.add_link('ListLink',
                     [atomspace.add_node('ObjectEntityNode', quad.subject, DEFAULT_TV),
                      atomspace.add_node('ObjectEntityNode', quad.destination, DEFAULT_TV)
                     ])
                ])
        elif quad.destination is None:
            atomspace.add_link('EvaluationLink',
                [atomspace.add_node('PredicateNode', quad.predicate, DEFAULT_TV),
                 atomspace.add_link('ListLink',
                     [atomspace.add_node('ObjectEntityNode', quad.subject, DEFAULT_TV),
                      atomspace.add_node(value_node_type, quad.value, DEFAULT_TV)
                     ])
                ])
        else:
            atomspace.add_link('EvaluationLink',
                [atomspace.add_node('PredicateNode', quad.predicate, DEFAULT_TV),
                 atomspace.add_link('ListLink',
                     [atomspace.add_node('ObjectEntityNode', quad.subject, DEFAULT_TV),
                      atomspace.add_link('EvaluationLink',
                          [atomspace.add_node('PredicateNode', 'is_key', DEFAULT_TV),
                           atomspace.add_link('ListLink',
                                [atomspace.add_node(value_node_type, quad.value, DEFAULT_TV),
                                 atomspace.add_node('ObjectEntityNode', quad.destination, DEFAULT_TV)
                                ])
                          ])
                     ])
                ])

if __name__ == '__main__':
    atomspace = AtomSpace()
    map_from_path('http://wiki.freebase.com/images/e/eb/Steve-martin-quad-sample.txt', atomspace)
    atomspace.print_list()
