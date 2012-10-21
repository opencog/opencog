__author__ = 'raminbarati'

"""INTERFACE"""
class IRunnable:
    def run(self):
        pass

"""INTERFACE"""
class IIncrementalLearner:
    def ConstructJoinTree(self, graph):
        pass

    def ConstructMpdTree(self, joinTree, moralizedGraph):
        pass

    def IncrementalCompilation(self, modificationList):
        pass



# pretty self explanatory
"""INCOMPLETE"""
"""ABSTRACT"""
class IncrementalLearnerBase(object, IRunnable, IIncrementalLearner):
# ABSTRACT
    def ThinOutGraph(self, graph):
        # I need some papers on this one
        # Should e-mail ben for some papers
        # Better be implemented in a child class
        pass

    # INCOMPLETE
    def CliqueDecomposition(self, graph):
        # Bron and Kerbosch algorithm, version 2
        # its kinda abstract for now
        cliques = []
        R = []
        P = graph.nodes
        X = []

        BronKerbosch(R,P,X)

        return cliques

        def BronKerbosch(R,P,X):
            if len(P) == 0 and len(X) == 0:
                cliques.append(R)
                return

            u = (P.union(X)).choose()
            n_u = u.neighbours()
            for v in P.remove(n_u):
                n = v.neighbours()
                r = R.union(v)
                p = P.intersect(n)
                x = X.intersect(n)
                BronKerbosch(r,p,x)
                P = P.remove(v)
                X = X.union(v)


    def ConstructJoinTree(self, graph):
        graph.moralise()
        graph_t = graph.triangulate()

        graph_min = self.ThinOutGraph(graph_t)
        t_min = self.CliqueDecomposition(graph_min)

        return t_min
    # comment
    # INCOMPLETE
    def ConstructMpdTree(self, joinTree, moralizedGraph):
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
            # INCOMPLETE
    def IncrementalCompilation(self, modificationList):
        raise Exception("not implemented")
    # INCOMPLETE
    def ModifyMoralGraph(self, modification):
        raise Exception("not implemented")
    # INCOMPLETE
    def Connect(self, clusterTree, cluster_i, cluster_j):
        raise Exception("not implemented")
    # INCOMPLETE
    def MarkAffectedMpsesByRemoveLink(self, mps_y, mps_z, linkList):
        raise Exception("not implemented")
    # INCOMPLETE
    def RemoveNode(self, node, mps_x, mps_y):
        raise Exception("not implemented")
    # INCOMPLETE
    def AddNode(self, node):
        raise Exception("not implemented")
    # INCOMPLETE
    def MarkAffectedMpsesByAddLink(self, linkList):
        raise Exception("not implemented")
    # INCOMPLETE
    def run(self):
        raise Exception("not implemented")

# Incremental learner with MCM-M graph thinner
class IncrementalLearnerMcm_m(IncrementalLearnerBase):
    def ThinOutGraph(self, graph):
        raise Exception("not implemented")
