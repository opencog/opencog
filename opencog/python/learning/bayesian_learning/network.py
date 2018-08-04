from learning.bayesian_learning.util import subsets_of, subsets_of_len_two

__author__ = 'keyvan'


class Link(list):
    repr_separator_symbol = '--'

    def __init__(self, first_node=None, second_node=None):
        self.first_node = first_node
        self.second_node = second_node

    def __hash__(self):
        return hash(self.first_node) ^ hash(self.second_node)

    def __eq__(self, other):
        if (self.first_node, self.second_node) == (other.first_node, other.second_node):
            return True
        if (type(self), type(other)) != (DirectedLink, DirectedLink):
            if (self.first_node, self.second_node) == (other.second_node, other.first_node):
                return True
        return False

    def __repr__(self):
        return str(self.first_node) + self.repr_separator_symbol + \
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
        if type(self.network) is BayesianNetwork and \
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

    def add_neighbour(self, neighbour):
        link = Link(self, neighbour)
        self.links_to_neighbours.add(link)
        neighbour.links_to_neighbours.add(link)

        self._neighbours.add(neighbour)
        neighbour._neighbours.add(self)
        return link

    def remove_neighbour(self, neighbour):
        self._neighbours.remove(neighbour)
        # Backup
        self._backup._neighbours.add(neighbour)

    @property
    def edges(self):
        return self.incoming_links | \
               self.outgoing_links | self.links_to_neighbours

    @property
    def neighbours(self):
        return self.parents | self.children | self._neighbours

    def revert(self):
        self.parents |= self._backup.parents
        self.children |= self._backup.children
        self._neighbours |= self._backup._neighbours

    def __len__(self):
        return len(self.incoming_links) + \
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


class Network(object):
    def __init__(self):
        self._edges = set()
        self._vertices = dict()
        self._needs_update = True

    def invalidate(self):
        self._needs_update = True

    def add_node(self, node_name):
        node = Node(node_name, self)
        self[node_name] = node
        return node

    def add_link(self, node_1_name, node_2_name):
        node_1 = self[node_1_name]
        node_2 = self[node_2_name]
        node_1.add_child(node_2)
        self.invalidate()

    def add(self, node):
        self[node.name] = node
        self.invalidate()

    def remove(self, node):
        self._vertices.pop(node.name)
        self.invalidate()

    @property
    def edges(self):
        if self._needs_update:
            self._edges = set()
            for vertice in self:
                self._edges |= vertice.edges
                self._needs_update = False
        return self._edges

    def __getitem__(self, node_name):
        return self._vertices[node_name]

    def __setitem__(self, node_name, value):
        self._vertices[node_name] = value

    def __repr__(self):
        return 'Vertices = ' + repr(list(self)) + '\n' + \
               'Edges = ' + str(self.edges)

    def __iter__(self):
        return iter(self._vertices.values())


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
        result = []
        for node in self:
            for parent1, parent2 in subsets_of_len_two(node.parents):
                result.append(str(parent1.add_neighbour(parent2)))
        return result

    def triangulate(self):
        added_links = []
        junction_tree = JunctionTree()
        vertices = list(self._vertices.values())
        eliminated = set()
        while len(vertices) > 0:
            vertices.sort() # based on number of neighbours
            node = vertices.pop()
            for neighbour1, neighbour2 in subsets_of_len_two(node.neighbours):
                if neighbour1 in eliminated or neighbour2 in eliminated:
                    continue
                added_links.append(Link(neighbour1, neighbour2))
                clique = Clique(neighbour1, node, neighbour2, junction_tree)
            eliminated.add(self.pop(node))
        for node in eliminated:
            node.revert()
            self.add(node)
        junction_tree.connect_links()
        return junction_tree, added_links

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
        self.node_set = frozenset((node1, node2, node3))
        self.name = ' '.join([node.name for node in [node1, node2, node3]])
        self.edges = set()
        self.network = junction_tree
        if junction_tree is not None:
            junction_tree.add(self)

    # def connect_link(self, link):
    #     self.edges.add(link)
    #
    # #        if link.first_node not in link.second_node.neighbours:
    # #            link.first_node.add_neighbour(link.second_node)

    def __eq__(self, other):
        return self.node_set == other.node_set

    def __repr__(self):
        return repr(tuple(self))

    def __iter__(self):
        return self.node_set.__iter__()


class JunctionTree(Network):
    def connect_links(self):
        for clique_1 in self:
            for clique_2 in self:
                if clique_1.node_set != clique_2.node_set and len(clique_1.node_set & clique_2.node_set) > 0:
                    link = Link(clique_1, clique_2)
                    clique_1.edges.add(link)
                    clique_2.edges.add(link)
        self.invalidate()


#################test
if __name__ == '__main__':
    network = BayesianNetwork()
    network.add_node('A')
    network.add_node('B')
    network.add_node('C')
    network.add_node('D')
    network.add_node('E')
    network.add_node('F')
    network.add_node('G')
    network.add_node('H')

    network.add_link('A', 'B')
    network.add_link('A', 'C')
    network.add_link('B', 'D')
    network.add_link('C', 'E')
    network.add_link('C', 'G')
    network.add_link('D', 'F')
    network.add_link('E', 'F')
    network.add_link('E', 'H')
    network.add_link('G', 'H')

    # network.moralise()
    # print len(network['G'])

    print 'Original Bayes Net:'
    print network, '\n---------------\n'
    print network.moralise(), '\n---------------\n'
    jt, links = network.triangulate()
    print links, '\n---------------\n'
    print jt, '\n---------------\n'
