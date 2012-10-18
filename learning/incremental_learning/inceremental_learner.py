__author__ = 'raminbarati'

# pretty self explanatory
"""INCOMPLETE"""
class IncrementalLearner(object):
    def ThinOutGraph(self, graph):
        # I need some papers on this one
        # Should e-mail ben for some papers
        # Better be implemented in a child class
        pass

    def CliqueDecomposition(self, graph):
        # Bron and Kerbosch algorithm?
        # Should e-mail ben for this one, maybe some papers?
        # If there are any other algorithms, better be virtual
        pass

    def ConstructJoinTree(self, graph):
        graph.moralise()
        graph_t = graph.triangulate()

        graph_min = self.ThinOutGraph(graph_t)
        t_min = self.CliqueDecomposition(graph_min)

        return t_min

    def ConstructMpdTree(self, joinTree, moralisedGraph):
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

    def IncrementalCompilation(self, modificationList):
        raise Exception("not implemented")

    def ModifyMoralGraph(self, modification):
        raise Exception("not implemented")

    def Connect(self, clusterTree, cluster_i, cluster_j):
        raise Exception("not implemented")

    def MarkAffectedMpsesByRemoveLink(self, mps_y, mps_z, linkList):
        raise Exception("not implemented")

    def RemoveNode(self, node, mps_x, mps_y):
        raise Exception("not implemented")

    def AddNode(self, node):
        raise Exception("not implemented")

    def MarkAffectedMpsesByAddLink(self, linkList):
        raise Exception("not implemented")





