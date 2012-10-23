__author__ = 'raminbarati'

"""INTERFACE"""
class Runnable:
    def run(self):
        pass

"""INTERFACE"""
class IncrementalLearner:
    def construct_join_tree(self, graph):
        pass

    def construct_mpd_tree(self, joinTree, moralisedGraph):
        pass

    def incremental_compilation(self, modificationList):
        pass

# pretty self explanatory
"""ABSTRACT"""
"""INCOMPLETE"""
class IncrementalLearnerBase(object, Runnable, IncrementalLearner):

    def __init__(self, old_network, new_network):
        self.old_network = old_network
        self.new_network = new_network

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
        cliques = []

        def bron_kerbosch(R,P,X):
            if len(P) == 0 and len(X) == 0:
                cliques.append(R)
                return
            u = (P|X).choose()
            for v in P-u.neighbours:
                bron_kerbosch(R|v,P-v.neibours(),X-v.neibours())
                P = P - v.neibours()
                X = X - v.neibours()
        pass

    def construct_join_tree(self, graph):
        graph_m = self.moralize(graph)
        graph_t = graph.triangulate(graph_m)

        graph_min = self.thin_out_graph(graph_t)
        t_min = self.clique_decomposition(graph_min)

        return t_min

    def construct_mpd_tree(self, joinTree, moralisedGraph):
        for clique_i in joinTree:
            for clique_j in joinTree:
                if clique_i == clique_j:
                    continue
                if len(clique_i) != len(clique_j):
                    continue

                n = len(clique_i)
                k = 0

                for node in clique_i:
                    if node in clique_j:
                        k += 1

                if k != n-1:
                    continue

                # Here I should find i they're connected by an "incomplete separator"
                # no idea what that is

                raise Exception("not implemented")

    def incremental_compilation(self, modificationList):
        raise Exception("not implemented")

    def modify_moral_graph(self, modification):
        raise Exception("not implemented")

    def connect(self, clusterTree, cluster_i, cluster_j):
        raise Exception("not implemented")

    def mark_affected_mpses_by_remove_link(self, mps_y, mps_z, linkList):
        raise Exception("not implemented")

    def remove_node(self, node, mps_x, mps_y):
        raise Exception("not implemented")

    def add_node(self, node):
        raise Exception("not implemented")

    def mark_affected_mpses_by_add_link(self, linkList):
        raise Exception("not implemented")