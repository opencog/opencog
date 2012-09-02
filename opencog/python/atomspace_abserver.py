from viz_graph import Graph_Abserver
from opencog.atomspace import types, AtomSpace
import networkx as ax
from types_inheritance import types_graph, name_to_type, is_a
#from pprint import pprint
from collections import defaultdict
from m_util import log, Logger
from m_adaptors import FakeAtom
log.add_level(Logger.INFO)
log.use_stdout(True)

    
#from opencog.atomspace import Atom, types
class Atomspace_Abserver(Graph_Abserver):
    ''' @bug: when link B have two same children ---- link A, then only one link A is given in the picture
              It happened rarely and  you could discover it when look the edge order(in the picture) carefully.
    '''
    """ attention: not including isolate concpet node and empty link"""
    def __init__(self, a, e_types = ["Link"], n_types = ["Node", "Link"], inheritance = True):
        super(Atomspace_Abserver, self).__init__(a, e_types, n_types, inheritance)
        #self.valid_node_types += self.valid_edge_types
        self.no_type = { }

    def graph_info(self):
        '''docstring for graph_info''' 
        self.e_types = ["Link"]
        self.n_types = ["Node", "Link"]
        edges_info = defaultdict(int)
        nodes_info = defaultdict(set)

        for e_type in self.valid_edge_types:
            links = self._get_edges(e_type)
            for link in links:
                nodes = self._nodes_from_edge(link)
                if len(nodes) > 0:
                    if self.valid_edge(link,nodes):
                        #edges_info[link.type_name].add(link.name)
                        edges_info[link.type_name] += 1
                        for i, node in enumerate(nodes):
                            if is_a(node.type_name, "Node"):
                                nodes_info[node.type_name].add(node.name)
        log.info("*******************************edges:" )
        for type_name, num in edges_info.iteritems():
            log.info( type_name + ":  " + str(num))
            #pprint(edges)
        log.info("*******************************nodes:" )
        for type_name, nodes in nodes_info.iteritems():
            log.info(str(len(nodes)),type_name)
            #pprint(nodes)
    
    def _get_edges(self,type):
        """docstring for __getEdges"""
        return  self.source.get_atoms_by_type(name_to_type(type))

    def _nodes_from_edge(self,edge):
        return edge.out

    def _edge_type(self,edge):
        return edge.type_name

    def _node_type(self,node):
        return node.type_name
	
    def _edge_is_a(self, source, target):
        if self.inheritance:
            return is_a(source,target)
        else:
            return source == target

    def _node_is_a(self, source, target):
        return ax.has_path(types_graph, target, source)

    def filter_graph(self):
        ''' get filtered graph from source''' 
        # add edges of valid type
        # iterate over valid edges
        for e_type in self.valid_edge_types:
            links = self._get_edges(e_type)
            for link in links:
                nodes = self._nodes_from_edge(link)
                # none empty edges!
                if len(nodes) > 0:
                    if self.valid_edge(link,nodes):
                        # make the linkname uniqueness
                        link_name = link.type_name + "[%s]"% str(link.h.value())
                        #print link_name
                        for i, node in enumerate(nodes):
                            if is_a(node.type_name, "Link"):
                               node_name = node.type_name + "[%s]"% str(node.h.value())
                               #print "***%s" % node_name
                            else:
                                node_name = self.graph.unique_id(node.name)
                                #print "^^%s" % node_name
                            #print "%s -> %s" %(link_name,node_name)
                            self.graph.add_edge(link_name,node_name)
                            # maintain order in the list
                            self.graph.set_edge_attr(link_name, node_name, order = str(i))

                            atom = FakeAtom(node.type, node_name, node.tv, node.av)
                            self.graph.set_node_attr(node_name, atom = atom)
                            #self.graph.set_node_attr(node_name, shape = "point")

                            atom = FakeAtom(link.type, link.name, link.tv, link.av)
                            self.graph.set_node_attr(link_name, atom = atom)
                            #self.graph.set_node_attr(link_name, shape = "point")
        return self.graph

if __name__ == '__main__':
    from load_scm_file import load_scm_file
    a = AtomSpace()
    load_scm_file(a, "./test/scm/test_atomspace_abserver.scm")
    abserver = Atomspace_Abserver(a)
    abserver.graph_info()
    abserver.filter_graph()
    abserver.write("test_atomspace_abserver.dot")

    
