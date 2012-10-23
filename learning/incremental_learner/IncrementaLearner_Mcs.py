__author__ = 'raminbarati'

import incremental_learner as incLer
import networkx as nx

class IncrementalLearner_Mcs(incLer.IncrementalLearnerBase):

    def triangulate(self, graph):
        import operator
        gt = graph.copy()
        F = set()
        weights = dict()
        numbered = set()

        def check(y,z):
            paths = nx.all_simple_paths(graph,y,z)

            for path in paths:
                if len(numbered.intersection(path)) > 0:
                    continue

                passed = True
                for x in path:
                    if x == y:
                        continue

                    if weights[x] >= weights[y]:
                        passed = False
                        break

                if not passed:
                    continue

                return True

            return False

        for node in graph:
            weights[node] = 0

        for k in range(1,len(graph),1):
            ordered_weights = sorted(weights.iteritems(), key=operator.itemgetter(1))
            print ordered_weights
            z = ordered_weights.pop()[0]
            while z in numbered:
                z = ordered_weights.pop()[0]

            for y in graph:
                if y == z:
                    continue
                if y in numbered:
                    continue

                if check(y,z):
                    weights[y] += 1
                    temp = (y,z)
                    F.add(temp)

            numbered.add(z)
        gt.add_edges_from(F)
        return gt

    #---------------test case-------------------



if __name__ == "__main__":

    old_graph = nx.DiGraph()
    old_graph.add_edges_from([('A','T'),('T','E'),('E','X'),('S','L'),('S','B'),('L','E'),('B','D'),('E','D')])

    new_graph = nx.DiGraph()
    new_graph.add_edges_from([('A','T'),('T','E'),('E','X'),('S','L'),('S','B'),('L','E'),('E','D')])

    il = IncrementalLearner_Mcs(old_graph,new_graph)
    gm = il.moralize(old_graph)

    gt = il.triangulate(gm)

    print '\n',gt.nodes()
    print gt.edges()