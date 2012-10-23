__author__ = 'raminbarati'

import incremental_learner as incLer
import networkx as nx

class IncrementalLearner_Mcs(incLer.IncrementalLearnerBase):

    def triangulate(self, graph):
        gt = nx.Graph()

        import operator
        unnumbered = set(graph)
        numbered = set()
        result = []

        weight = dict()
        for node in graph:
            weight[node] = 0

        while unnumbered:
            sorted_weights = sorted(weight.iteritems(), key=operator.itemgetter(1))

            v = sorted_weights.pop()[0]
            while v in numbered:
                v = sorted_weights.pop()[0]

            result.insert(0,v)
            unnumbered.remove(v)

            for n in graph[v]:
                weight[n] += 1

            numbered.add(v)

        processed = set()
        for node in result:
            neigh = set(graph[node])
            wanna_be_clique = neigh.difference(processed)
            wanna_be_clique.add(node)

            for i in range(0,len(wanna_be_clique),1):
                node_i = wanna_be_clique.pop()
                for node_j in wanna_be_clique:
                    gt.add_edge(node_i,node_j)

            processed.add(node)

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
    t_min = il.construct_join_tree(old_graph)

    print len(t_min)

    for n in t_min.nodes_iter():
        print t_min[n]