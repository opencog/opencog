__author__ = 'raminbarati'
import networkx as nx
from util import switch
from modification import Modification

class Runnable:
    """INTERFACE"""
    def run(self):
        pass

class IncrementalLearner:
    """INTERFACE"""
    def construct_join_tree(self, graph):
        pass

    def construct_mpd_tree(self, joinTree, moralisedGraph):
        pass

    def incremental_compilation(self, modificationList):
        pass

# pretty self explanatory
class IncrementalLearnerBase(object, Runnable, IncrementalLearner):
    """ABSTRACT, INCOMPLETE"""
    def __init__(self, old_network):
        self._old_network = old_network
        self._graph_m = None
        self._jt = None
        self._jt_mpd = None
        self._initialized = False
        self._marked = []

    def moralize(self,directed_graph):
        gm = directed_graph.to_undirected()
        for node in directed_graph.nodes_iter():
            pres = directed_graph.predecessors(node)
            for i in range(0,len(pres),1):
                for j in range(i+1,len(pres),1):
                    gm.add_edge(pres[i],pres[j])
        return gm

#   input graph should not be directed, use moralize first
    def triangulate(self, graph):
        # implemented in a child class
        pass

    def thin_out_graph(self, graph):
        # override in a child class if triangulation isn't minimal
        return graph

    def clique_decomposition(self, graph):
        cluster_graph = nx.Graph()
        cliques = list(graph.subgraph(c) for c in nx.find_cliques(graph))

        while cliques:
            clique_i = cliques.pop()
            for clique in cliques:
                clique_j = clique
                shared = set(clique_i).intersection(set(clique_j))
                if len(shared) > 0:
                    cluster_graph.add_edge(clique_i, clique_j, {'label':shared, 'weight':1.0/len(shared)})

        j_tree = nx.minimum_spanning_tree(cluster_graph)

        return j_tree

    def construct_join_tree(self, graph):
        graph_m = self.moralize(graph)
        graph_t = self.triangulate(graph_m)
        graph_min = self.thin_out_graph(graph_t)
        jt_min = self.clique_decomposition(graph_min)

        return jt_min

    def construct_mpd_tree(self, jt_min, graph_m):
        def is_complete(nbunch):
            sub_g = graph_m.subgraph(nbunch)
            n = len(nbunch)
            if n == 1:
                return True
            m = sub_g.size()
            if n*(n-1)/2 == m:
                return True
            return False

        def aggregate(node_i,node_j):
            union = set(node_i).union(set(node_j))
            sub_g = graph_m.subgraph(union)
            jt_mpd.add_node(sub_g)
            sub_g_n = set(sub_g)
            neigh = set(jt_mpd[node_i]).union(jt_mpd[node_j])
            for n_i in neigh:
                sep = set(n_i).intersection(sub_g_n)
                jt_mpd.add_edge(n_i,sub_g, {'label':sep})
            jt_mpd.remove_node(node_i)
            jt_mpd.remove_node(node_j)

        jt_mpd = jt_min.copy()
        while True:
            nodes = jt_mpd.nodes()
            complete = True
            for node in nodes:
                for neighbor in jt_mpd[node]:
                    seperator = jt_mpd[neighbor][node]['label']
                    if not is_complete(seperator):
                        complete = False
                        aggregate(neighbor,node)
                        break
                if not complete:
                    break
            if complete:
                break
        return jt_mpd

    def incremental_compilation(self, modificationList):
        for modification in modificationList:
            L = self.modify_moral_graph(modification)

            for case in switch(modification.type):
                if case(Modification.ADD_NODE):
                    self.add_node(modification.data)
                    break
                if case(Modification.REMOVE_NODE):
                    self.remove_node(modification.data)
                    break
                if case(Modification.ADD_LINK):
                    self.mark_affected_mps_by_add_link(L)
                    break
                if case(Modification.REMOVE_LINK):
                    self.mark_affected_mps_by_remove_link(None,None,L)
                    break
                if case():
                    pass
        raise Exception("not implemented")

    def modify_moral_graph(self, modification):
        L = []

        for case in switch(modification.type):
            if case(Modification.ADD_NODE):
                self._graph_m.add_node(modification.data)
                break
            if case(Modification.REMOVE_NODE):
                self._graph_m.remove_node(modification.data)
                break
            if case(Modification.ADD_LINK):
                pair = set(modification.data)
                parents = set(self._old_network.predecessors(modification.data[1]))
                nodes = pair.union(parents)
                subgraph = self._graph_m.subgraph(nodes)
                complement = nx.complement(subgraph)
                for edge in complement.edges_iter():
                    L.append(edge)
                break
            if case(Modification.REMOVE_LINK):
                head = modification.data[1]
                tail = modification.data[0]

                children_head = set(self._old_network.successors(head))
                children_tail = set(self._old_network.successors(tail))

                if len(children_tail.intersection(children_head)) <= 0:
                    self._graph_m.remove_edge(modification.data)
                    L.append(modification.data)

                for parent in self._old_network.predecessors_iter(head):
                    if parent == tail: continue
                    children_z_i = set(self._old_network.successors(parent)).intersection(children_tail)

                    if not len(children_z_i) == 1: continue
                    if head not in children_z_i: continue
                    if not self._old_network.has_edge(parent,tail): continue
                    if self._old_network.has_edge(tail, parent): continue

                    self._graph_m.remove_edge(tail,parent)
                    L.append((tail,parent))
                break
            if case():
                raise Exception('Not a defined modification')

        return L

    def connect(self, clusterTree, cluster_i, cluster_j):
        raise Exception("not implemented")

    def mark_affected_mps_by_remove_link(self, mps_y, mps_z, linkList):
        raise Exception("not implemented")

    def remove_node(self, node, mps_x = None, mps_y = None):
        raise Exception("not implemented")

    def add_node(self, node):
        C_x = nx.Graph()

        C_x.add_node(node, {'marked':False})
        self._jt.add_node(C_x)
        self._jt_mpd.add_node(C_x)

    def mark_affected_mps_by_add_link(self, linkList):
        raise Exception("not implemented")