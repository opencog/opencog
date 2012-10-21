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
"""INCOMPLETE"""
class IncrementalLearnerBase(object, Runnable, IncrementalLearner):
    def thin_out_graph(self, graph):
        # I need some papers on this one
        # Should e-mail ben for some papers
        # Better be implemented in a child class
        pass

    def clique_decomposition(self, graph):
        # Bron and Kerbosch algorithm?
        # Should e-mail ben for this one, maybe some papers?
        # If there are any other algorithms, better be virtual
        pass

    def construct_join_tree(self, graph):
        graph.moralise()
        graph_t = graph.triangulate()

        graph_min = self.ThinOutGraph(graph_t)
        t_min = self.CliqueDecomposition(graph_min)

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