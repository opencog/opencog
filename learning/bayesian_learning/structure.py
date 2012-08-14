from learning.bayesian_learning.network import *
from random import randrange, random as rand
from util.evolutionary import *
from math import log, factorial

__author__ = 'keyvan'


class BayesNetPopulation(Population):

    def __init__(self, variable_names, number_of_individuals=0):
        self.scoring_function = K2(variable_names)
        Population.__init__(self, NetworkChromosomeRepresentation, number_of_individuals,
            variable_names=variable_names, scoring_function=self.scoring_function)

    def process_new_data(self, data):
        self.scoring_function.process_new_data(data)

class ScoringFunction(object):
    def __init__(self, variable_names, data):
        self.variable_names = variable_names
        self.data =  data

    def probability_of(self, network):
        return 1

    def process_new_data(self, network, data):
        pass

#    def __call__(self, individual):
#        if individual.last


class BinaryVariableScoringFunction(ScoringFunction):
    pass


def matches(row, config):
    if len(config) is 0 and len(row) is not 0:
        return False
    return config.issubset(row)


class K2(BinaryVariableScoringFunction):

    def process_new_data(self, network, data):
        P = self.probability_of
        X = self.variable_names
        B = network
        T = data
        x_i = [True, False]
        r_i = 2
        N = {}

        for data_row in T:
            for X_i in X:
                N[X_i] = {}
                if X_i in B.CPTs:
                    for w_ij, theta_ij in B.CPTs[X_i]:
                        N[X_i][w_ij] = {}
                        for x_ik in x_i:
                            N[X_i][w_ij][x_ik] = 0
                        if matches(data_row, w_ij):
                            for x_ik in x_i:
                                if data_row[X_i] is x_ik:
                                    N[X_i][w_ij][x_ik] += 1


        r_i_minus_one_fact = float(factorial(r_i-1))
        score = log(P(B))
        for X_i in X:
            if X_i in B.CPTs:
                for w_ij, theta_ij in B.CPTs[X_i]:
                    N_ij = 0
                    for x_ik in x_i:
                        N_ij += N[X_i][w_ij][x_ik]
                        score += log(factorial(N[X_i][w_ij][x_ik]))
                    score += log(r_i_minus_one_fact/factorial(N_ij + r_i - 1))
        return score


class NetworkChromosomeRepresentation(IndividualSetBase):

    def __init__(self, chromosome_to_copy_from=None):
        self.nodes = {}
        if chromosome_to_copy_from is None:
            self.network = BayesianNetwork()
        else:
            self.network = chromosome_to_copy_from.network.copy()
            for node in self.network:
                self.nodes[node.name] = node
            self |= self.network.edges

    scoring_function = None


    def __randomly_initialise__(self):
        for i in range(len(self.variable_names) / 2):
            self.add_random_link()

    def get_node_by_name(self, name):
        if not self.nodes.has_key(name):
            self.nodes[name] = Node(name, self.network)
        return self.nodes[name]

    def __setitem__(self, key, link):
        parent = self.get_node_by_name(link.first_node.name)
        child = self.get_node_by_name(link.second_node.name)
        new_link = parent.add_child(child)
        self.add(new_link)

    @property
    def _random_node(self):
        name = self.variable_names[randrange(0, len(self.variable_names))]
        return self.get_node_by_name(name)

    def add_random_link(self):
        for i in range(len(self.variable_names)):
            first_node = self._random_node
            while True:
                second_node = self._random_node
                if second_node.name != first_node.name:
                    break
            link = DirectedLink(first_node, second_node)
            if link not in self and self.network.check_dag(link):
                link.place_between(first_node, second_node)
                self.network.invalidate()
                self.add(link)
                return link

    def delete_random_link(self):
        if not len(self): # Network has no links
            return
        link = self.pop()
        link.remove()

    def reverse_random_link(self):
        if not len(self):
            return
        link = list(self)[randrange(0,len(self))]
        if self.network.check_dag(link.inverse):
            link.invert()
        return link

    def __fitness__(self):
        return self.scoring_function(self.network)

    def __mutate__(self):
        offspring = NetworkChromosomeRepresentation(self)
        num = rand()
        if num < float(1) / 3:
            offspring.add_random_link()
        elif num < float(2) / 3:
            offspring.reverse_random_link()
        else:
            offspring.delete_random_link()
        return offspring

if __name__ == '__main__':
    from util.evolutionary import GeneticAlgorithm
    from learning.bayesian_learning.dynamics import *

    data = DataObserver()
    for i in range(10):
        data.append(generate_test_record())

    population = BayesNetPopulation(TEST_VARIABLES, data, 10)


    ga = GeneticAlgorithm(population=population)
    ga.run()
