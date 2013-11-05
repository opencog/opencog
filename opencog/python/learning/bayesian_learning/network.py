from learning.bayesian_learning.util import subsets_of
from utility.generic import subsets_of_len_two

__author__ = 'keyvan'


class Link(list):

    repr_separator_symbol = '--'

    def __init__(self, first_node=None, second_node=None):
        self.first_node = first_node
        self.second_node = second_node

    def __hash__(self):
        return hash(self.first_node) ^ hash(self.second_node)

    def __eq__(self, other):
        if self.first_node == other.first_node and\
           self.second_node == other.second_node:
            return True
        if (type(self), type(other)) != (DirectedLink, DirectedLink):
            if self.first_node == other.second_node and\
               self.second_node == other.first_node:
                return True
        return False

    def __repr__(self):
        return str(self.first_node) + self.repr_separator_symbol +\
               str(self.second_node)


class DirectedLink(Link):

    repr_separator_symbol = '->'

    @property
    def inverse(self):
        return DirectedLink(self.second_node, self.first_node)

    def invert(self):
        self.remove()
        self.place_between(self.second_node, self.first_node)
        self.repr_separator_symbol = '--i>'

    def remove(self):
        self.first_node.children.remove(self.second_node)
        self.second_node.parents.remove(self.first_node)
        self.first_node.outgoing_links.remove(self)
        self.second_node.incoming_links.remove(self)

    def place_between(self, first_node=None, second_node=None):
        if not first_node or not second_node:
            first_node, second_node = self.first_node, self.second_node
        first_node.children.add(second_node)
        second_node.parents.add(first_node)
        self.first_node, self.second_node = first_node, second_node
        first_node.outgoing_links.add(self)
        second_node.incoming_links.add(self)

class Node(object):

    def __init__(self, name=None, host_network=None, backup=True):
        self.name = name
        self.incoming_links = set()
        self.outgoing_links = set()
        self.links_to_neighbours = set()
        self.parents = set()
        self.children = set()
        self._cpt = None
        self._neighbours = set()
        if backup:
            self._backup = Node(None, None, False)

        if host_network is not None:
            host_network.add(self)
        self.network = host_network

    @property
    def CPT(self):
        if self._cpt is None:
            self._cpt = ConditionalProbabilityTable(self)
        return self._cpt

    def add_parent(self, parent):
        return self.add_child(parent, self)

    def add_child(self, child):
        """
        returns None if adding link is at odds with
        network's being DAG requirement
        """
        link = DirectedLink(self, child)
        if type(self.network) is BayesianNetwork and\
           self.network.check_dag(link):
            link.place_between()
            return link
        return None

    def remove_parent(self, parent):
        self.parents.remove(parent)
        link = DirectedLink(parent, self)
        self.incoming_links.remove(link)
        # Backup
        self._backup.parents.add(parent)
        self._backup.incoming_links.add(link)

    def remove_child(self, child):
        self.children.remove(child)
        link = DirectedLink(self, child)
        self.outgoing_links.remove(link)
        # Backup
        self._backup.children.add(child)
        self._backup.outgoing_links.add(link)

    def add_neighbour(self, neighbour, __link=None):
        if __link is None:
            __link = Link(self, neighbour)
            neighbour.add_neighbour(self, __link)
        self.links_to_neighbours.add(__link)
        self._neighbours.add(neighbour)

    def remove_neighbour(self, neighbour):
        self._neighbours.remove(neighbour)
        # Backup
        self._backup._neighbours.add(neighbour)

    @property
    def edges(self):
        return self.incoming_links |\
               self.outgoing_links | self.links_to_neighbours

    @property
    def neighbours(self):
        return self.parents | self.children | self._neighbours

    def revert(self):
        self.parents |= self._backup.parents
        self.children |= self._backup.children
        self._neighbours |= self._backup._neighbours

    def __len__(self):
        return len(self.incoming_links) +\
               len(self.outgoing_links) + len(self._neighbours)

    def __eq__(self, other):
        return self.name == other.name

    def __cmp__(self, other):
        return len(other) - len(self)

    def __repr__(self):
        if self.name is None:
            return object.__repr__(self)
        return self.name

class Row(set):
    def __getitem__(self, variable_name):
        if variable_name in self:
            return True
        return False

    def __hash__(self):
        _hash = 0
        for item in self:
            _hash ^= hash(item)
        return _hash



class ConditionalProbabilityTable(dict):

    def __init__(self, node=None):
        self.node = node
        self.parents = set([parent.name for parent in node.parents])

    def __getitem__(self, config):
        if not config in self:
            return 0
        return self[config]

    def __iter__(self):
        for config in subsets_of(self.parents, Row):
            yield config, self[config]


class Graph(object):

    def add(self, obj):
        pass

    def remove(self,obj):
        pass


    def vertices(self):
        pass
    def edges(self):
        pass


class Network(Graph):

    def __init__(self):
        self._edges = set()
        self._needs_update = True

    def invalidate(self):
        self._needs_update = True

    @property
    def edges(self):
        if self._needs_update:
            self._edges = set()
            for vertice in self:
                self._edges |= vertice.edges
                self._needs_update = False
        return self._edges

    def __repr__(self):
        return 'Vertices = ' + repr(list(self)) + '\n' +\
               'Edges = ' + str(self.edges)



#class DAGOrganiser(object):
#    def __init__(self):
#        self.paths_by_first_node = {}
#        self.paths_by_last_node = {}
#
#    def check_link(self, link):
#        paths = self.paths_by_node(link.first_node)
#        for path in paths:
#            for other_link in path:
#                if other_link.first_node == link.second_node:
#                    return False
#        return True
#
#    def notify_link_removal(self, link):
#        pass
#
#    def notify_link_addition(self, link):
#        paths_with_node_at_end_exist = link.second_node in self.paths_by_last_node
#        paths_with_node_at_beginning_exist = link.first_node in self.paths_by_first_node
#
#        condition = (paths_with_node_at_end_exist,
#                     paths_with_node_at_beginning_exist)
#
#        if condition is False, False:
#            path = [link]
#            self.paths_by_last_node[link.socond_node] = [path]
#            self.paths_by_first_node[link.first_node] = [path]
#        elif condition is True, False:
#            self._add_link(link, -1)
#        elif condition is False, True:
#            self._add_link(link, 0)
#        else: # True, True
#            self.merge_paths(link)
#
#    def _add_link(self, link, index):
#        nodes = [link.first_node, link.second_node]
#        node_to_append_paths_to = nodes.pop(index)
#        node_to_read_paths_from = nodes.pop()
#        paths = [self.paths_by_first_node, self.paths_by_last_node][index]
#        if node_to_append_paths_to not in paths:
#            paths[node_to_append_paths_to] = []
#
#        for path in paths[node_to_read_paths_from]:
#            node_to_remove_path_from = path[index][index]
#            paths[node_to_remove_path_from].remove(path)
#            path.append(link)
#            paths[node_to_append_paths_to].append(path)
#
#    def merge_paths(self, link):
#        pass


class BayesianNetwork(Network):

    def __init__(self):
        Network.__init__(self)

    def create_node(self, name):
        node = Node(name, self)
        self.add(node)
        return node

    @property
    def CPTs(self):
        _cpts = {}
        for node in self:
            _cpts[node.name] = node.CPT
        return _cpts


    def add_range_nodes(self, nodes):
        self |= nodes

    def pop(self, node):
        none_moral_neighbours = set()
        for parent in node.parents:
            parent.remove_child(node)
            none_moral_neighbours.add(parent)
        for child in node.children:
            child.remove_parent(node)
            none_moral_neighbours.add(child)
        for moral_neighbour in node.neighbours - none_moral_neighbours:
            node.remove_neighbour(moral_neighbour)
        self.remove(node)
        return node

    def moralise(self):
        for node in self:
            for parent1, parent2 in subsets_of_len_two(node.parents):
                parent1.add_neighbour(parent2)

    def triangulate(self):
        junction_tree = JunctionTree()
        vertices = list(self)
        eliminated = set()
        while len(vertices) > 0:
            vertices.sort() # based on number of neighbours
            node = vertices.pop()
            for neighbour1, neighbour2 in subsets_of_len_two(node.neighbours):
                if neighbour1 in eliminated or neighbour2 in eliminated:
                    continue
                link = neighbour1.add_neighbour(neighbour2)
                clique = Clique(neighbour1, node, neighbour2, junction_tree)
#                clique = Clique(neighbour1)
                clique.connect_link(link)
            eliminated.add(self.pop(node))
        for node in eliminated:
            node.revert()
            self.add(node)
        return junction_tree

    def check_dag(self, link):
        parents = link.first_node.parents.copy()
        while len(parents):
            node = parents.pop()
            if node == link.second_node:
                return False
            parents |= node.parents
        return True

    def __dim__(self):
        dim = 0
        for node in self:
            if len(node.parents) is 0:
                dim += 1
        return dim

    def copy(self):
        """
        Deep copy
        """
        clone = BayesianNetwork()
        nodes_by_name = {}
        for node in self:
            nodes_by_name[node.name] = Node(node.name)
            clone.add(node)
        for link in self.edges:
            if link is DirectedLink:
                new_link = DirectedLink()
                new_link.place_between(nodes_by_name[link.first_node],
                    nodes_by_name[link.second_node])
        return clone


class Clique(object):

    def __init__(self, node1, node2, node3, junction_tree=None):
        self.frozenSet = frozenset((node1, node2, node3))
        self.edges = set()
        self.network = junction_tree
        if junction_tree is not None:
            junction_tree.add(self)

    def connect_link(self, link):
        self.edges.add(link)
#        if link.first_node not in link.second_node.neighbours:
#            link.first_node.add_neighbour(link.second_node)

    def __repr__(self):
        return repr(tuple(self))

    def __iter__(self):
        return self.frozenSet.__iter__()


class JunctionTree(Network):
    pass



#################test
if __name__ == '__main__':
    network = BayesianNetwork()
    A = Node('A', network)
    B = Node('B', network)
    C = Node('C', network)
    D = Node('D', network)
    E = Node('E', network)
    F = Node('F', network)
    G = Node('G', network)
    H = Node('H', network)
    A.add_child(B)
    A.add_child(C)
    B.add_child(D)
    C.add_child(E)
    C.add_child(G)
    D.add_child(F)
    E.add_child(F)
    E.add_child(H)
    G.add_child(H)

    print 'Original Bayes Net:'
    print network, '\n---------------\n'
    network.moralise()
    print network, '\n---------------\n'
    jt = network.triangulate()
    print jt, '\n---------------\n'

    l1 = DirectedLink(A,B)
    l2 = Link(B,A)

    print l1 == l2

    a = set(); a.add(l1); a.add(l2)
    print a
    print hash(l1)
    print hash(l2)


    #comajhsb