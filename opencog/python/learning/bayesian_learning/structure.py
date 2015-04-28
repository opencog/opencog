from learning.bayesian_learning.network import *
from random import randrange, random as rand
from learning.bayesian_learning.util import dim
from sample_data.uci_adult_dataset.main import uci_adult
from utility.evolutionary import *
from utility.numeric.information_theory import mutual_information
from math import log, factorial

__author__ = 'keyvan'


class BayesNetPopulation(Population):

    def __init__(self, data, number_of_individuals=0):
        variable_names = data.variable_names
        self.scoring_function = BayesianInformationCriterionScore(variable_names, data)
        Population.__init__(self, NetworkChromosomeRepresentation, number_of_individuals,
            variable_names=variable_names, scoring_function=self.scoring_function)

    def process_new_data(self, data):
        self.scoring_function.process_new_data(data)

class ScoringFunction(object):
    def __init__(self, variable_names, data):
        self.variable_names = variable_names
        self.data =  data

    def __call__(self, network):
        return 1

class BayesianInformationCriterionScore(ScoringFunction):
    def __call__(self, network):
        score = 0
        M = float(len(self.data))
        for node in network:
            for parent in node.parents:
                score += mutual_information(self.data, node.name, parent.name)
        score *= M
        score -= log(M)/2 * dim(network) # penalise complexity
        return score


class NetworkChromosomeRepresentation(IndividualSetBase):

    def __init_normal__(self):
        self.nodes = {}
        self.network = BayesianNetwork()
        #randomly initialise
        for i in range(len(self.variable_names) / 2):
            self.add_random_link()

    def __init_by_parent__(self, parent):
        self.nodes = {}
        self.network = parent.network.copy()
        for node in self.network:
            self.nodes[node.name] = node
        self |= self.network.edges

    scoring_function = None

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
        offspring = new_offspring(self)
        num = rand()
        if num < float(1) / 3:
            offspring.add_random_link()
        elif num < float(2) / 3:
            offspring.reverse_random_link()
        else:
            offspring.delete_random_link()
        return offspring

if __name__ == '__main__':
    from utility.evolutionary import GeneticAlgorithm

    data = uci_adult()

    population = BayesNetPopulation(data , 10)

    ga = GeneticAlgorithm(population=population)
    ga.run()
