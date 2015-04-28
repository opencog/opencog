__author__ = 'raminbarati'

import incremental_learner as incLer
import networkx as nx

class IncrementalLearnerMcsMinimal(incLer.IncrementalLearnerBase):

    def triangulate(self, graph):
        def check(start,goal):
#            return True
            def is_goal(node):
                return node == goal
            def get_successors(node):
                successors = set()
                for successor in graph[node]:
                    if not successor == goal:
                        if successor in numbered:
                            continue
                        if weight[successor] >= w_min:
                            continue
                    successors.add(successor)
                return successors
            #------------------------
            w_min = min(weight[start],weight[goal])
            if start == goal:
                return False
            expandedNodes = set()
            fringe = []
            fringe.insert(0,start)
            expandedNodes.add(start)

            while fringe:
                t = fringe.pop()
                if is_goal(t):
                    return True

                childes = get_successors(t)

                for child in childes:
                    if child in expandedNodes:
                        continue
                    else:
                        expandedNodes.add(child)

                    fringe.insert(0,child)

            return False
        #----------------------
        gt = graph.copy()

        import operator
        unnumbered = set(graph)
        numbered = set()
        result = []

        weight = dict()
        for node in graph:
            weight[node] = 0

#        weight['B'] = 1

        while unnumbered:
            passed = []
            sorted_weights = sorted(weight.iteritems(), key=operator.itemgetter(1))
            v = sorted_weights.pop()[0]
            while v in numbered:
                v = sorted_weights.pop()[0]
            result.insert(0,v)
            unnumbered.remove(v)
            for n in unnumbered:
                if check(n,v):
                    passed.append(n)
                    gt.add_edge(n,v)
            for p in passed:
                weight[p] += 1
            numbered.add(v)
        return gt





#---------------test case-------------------
if __name__ == "__main__":

    old_graph = nx.DiGraph()
    old_graph.add_edges_from([('A','T'),('T','E'),('E','X'),('S','L'),('S','B'),('L','E'),('B','D'),('E','D')])

    new_graph = nx.DiGraph()
    new_graph.add_edges_from([('A','T'),('T','E'),('E','X'),('S','L'),('S','B'),('L','E'),('E','D')])

    il = IncrementalLearnerMcsMinimal(old_graph)
    gm = il.moralize(old_graph)
#    gt = il.triangulate(gm)

    t_min = il.construct_join_tree(old_graph)
#    print 'Nodes of junction tree:'
#    for n in t_min.nodes_iter():
#        print n.nodes()
#    print 'Edges of junction tree:'
#    for e in t_min.edges_iter():
#        print (e[0].nodes(),e[1].nodes())

    t_mpd = il.construct_mpd_tree(t_min,gm)
    print 'Nodes of junction tree:'
    for n in t_mpd.nodes_iter():
        print n.nodes()
    print 'Edges of junction tree:'
    for e in t_mpd.edges_iter():
        print (e[0].nodes(),e[1].nodes())