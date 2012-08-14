__author__ = 'keyvan'

from random import randrange, random as rand
from util.generic import new_instance_of_same_type as new_offspring

class IndividualBase(object):
    """
    An individual object should be a collection of Genes,
    each gene should be accessible via a 'locus', so if
    individual is implemented in terms of dic, locus is
    key and
    """

    #############################################################
    ## User of this code should implement the following.
    ## Refer to examples/genetic_algorithm for demo

    loci = None
    _out_of_date_fitness_value = True
    population = None

    def __init__(self, **common_attributes):
        """
        Do not override init, every key in **common_attributes
        will be an attribute of the individual object automatically.
        You can override __randomly_initialise__ which is called
        after __init__
        """
        self.__dict__.update(common_attributes)
        self.__randomly_initialise__()

    def __fitness__(self):
        pass

    def __mutate__(self):
        pass

    def __crossover__(self, other):
        return self.fitness_proportionate_crossover(other)

    def __getitem__(self, key):
        pass

    def __setitem__(self, key, value):
        pass

    def __randomly_initialise__(self):
        """
        Override this method if you want to randomly initialise
        the individual before it's added to the population
        """
        pass

    #############################################################

    @property
    def fitness(self):
        if self._out_of_date_fitness_value:
            self._fitness = self.__fitness__()
            self._out_of_date_fitness_value = False
        return self._fitness

    def mutate(self):
        self._out_of_date_fitness_value = True
        return self.__mutate__()

    def __add__(self, other): # + operator does crossover
        return self.__crossover__(other)

    def __cmp__(self, other):
        return other.fitness - self.fitness
#        return self.fitness - other.fitness

    #############################################################
    ## Predefined crossover methods
    def proportionate_crossover(self, other, self_share):
        offspring = new_offspring(self)
        for locus in set(self.loci) | set(other.loci):
            if rand() < self_share:
                if locus not in self.loci:
                    continue
                offspring[locus] = self[locus]
            else:
                if locus not in other.loci:
                    continue
                offspring[locus] = other[locus]
        return offspring

    def uniform_crossover(self, other):
        return self.proportionate_crossover(other, 0.5)

    def fitness_proportionate_crossover(self, other):
        if self.fitness == 0 and other.fitness == 0:
            self_share = 0.5
        else:
            self_share = float(self.fitness) / (self.fitness + other.fitness)
        return self.proportionate_crossover(other, self_share)

    def one_point_crossover(self, other, point_index):
        pass # TODO

    def two_point_crossover(self, other, first_point_index = None,
                            second_point_index = None):
        pass # TODO
    #############################################################


class Population(object):

    def __init__(self, type_of_individuals, number_of_individuals=0,
                 **common_attributes_between_individuals):
        self.common_attributes = common_attributes_between_individuals
        self.current_generation = []
        self.next_generation = []
        self.type_of_individuals = type_of_individuals
        self.generation_count = 0
        self.add_many(number_of_individuals)

    def _assign_common_attributes(self, individual):
        individual.population = self
        individual.__dict__.update(self.common_attributes)

    def __selection__(self):
        """
        Override this method to control selection behaviour.
        Select method should return one individual.
        The default implementation returns an individual from the
        fitter half of the population. Population is sorted in
        the entry point of this method.
        """
        return self.current_generation[randrange(0, len(self.current_generation)/2)]

    def __crossover_selection__(self):
        """
        Override if your crossover selection method is
        different from your mutation selection.
        """
        return self.__selection__(), self.__selection__()

    def select_for_mutation(self):
        offspring =  self.__selection__()
        self._assign_common_attributes(offspring)
        return offspring

    def select_for_crossover(self):
        return self.__crossover_selection__()

    def add_many(self, quantity):
        for _ in range(quantity):
            individual = self.type_of_individuals() # new instance
            self.add_to_current_generation(individual)

    def add_to_current_generation(self, individual):
        self.current_generation.append(individual)

    def add_to_next_generation(self, offspring):
        self.next_generation.append(offspring)

    def switch_to_next_generation(self):
        self.current_generation = self.next_generation
        self.next_generation = []
        self._sorted = False
        self.generation_count += 1

    def sort(self):
        self.current_generation.sort()

    def __len__(self):
        return len(self.current_generation)

    def __getitem__(self, index):
        return self.current_generation[index]

class GeneticAlgorithm(object):

    sort_population_each_step = True

    def __init__(self, **kwargs):
        """
        Two ways for initialising:
        1) giving the population by passing 'population' parameter
        2) specifying 'type_of_individuals' and 'type_of_individuals'
        """
        self.__dict__.update(kwargs)
        if not self.__dict__.has_key('population'):
            if not self.__dict__.has_key('type_of_individuals')\
            or not self.__dict__.has_key('number_of_individuals'):
                raise ValueError('since population is not specified,'
                                 ' type_of_individuals and number_of_individuals'
                                 ' should be present')
            self.population = Population(self.type_of_individuals, self.number_of_individuals)
        self.highest_fitness_found = 0

    def step(self, mutation_rate=1, crossover_rate = 1,
             number_of_individuals=0):
        highest_fitness_this_generation = 0
        fittest_individual_this_generation = None
        if number_of_individuals <= 0:
            number_of_individuals = len(self.population)
        while len(self.population.next_generation) < number_of_individuals:
            if self.sort_population_each_step:
                self.population.sort()
            if 0 < crossover_rate > rand():
                parent, other_parent = self.population.select_for_crossover()
                offspring = parent + other_parent # crossover
            else:
                offspring = self.population.select_for_mutation()
            if 0 < mutation_rate > rand():
                offspring = offspring.mutate()
            self.population.add_to_next_generation(offspring)
            if offspring.fitness > highest_fitness_this_generation:
                fittest_individual_this_generation = offspring
                highest_fitness_this_generation = offspring.fitness

        if highest_fitness_this_generation > self.highest_fitness_found:
            self.highest_fitness_found = fittest_individual_this_generation.fitness
            self.fittest_individual_found = fittest_individual_this_generation
        self.population.switch_to_next_generation()
        return fittest_individual_this_generation

    def run(self, show_population_each_generation=True):
        while True:
            if show_population_each_generation:
                print '#################### Generation ' +\
                      str(self.population.generation_count) +\
                      ' ####################'
                for individual in self.population:
                    print individual
            print 'Fittest:', str(self.step())

class IndividualListBase(list, IndividualBase):
    @property
    def loci(self):
        return range(len(self))

class IndividualDictBase(dict, IndividualBase):
    @property
    def loci(self):
        return self

class IndividualSetBase(set, IndividualBase):

    @property
    def loci(self):
        return self

    def __getitem__(self, item):
        return item

    def __setitem__(self, key, value):
        self.remove(key)
        self.add(value)


class NoneEpistaticGeneticAlgorithm(GeneticAlgorithm):

    fitness_unit = 1

    class _contribution_dict(dict):
        def __getitem__(self, item):
            if item not in self:
                return NoneEpistaticGeneticAlgorithm.fitness_unit
            return dict.__getitem__(self, item)

    fitness_contribution_by_locus = _contribution_dict()

    def __init__(self, type_of_individuals, number_of_individuals):
        self.population = _none_epistatic_population(type_of_individuals, number_of_individuals)
        self.population.a

    def step(self, mutation_rate=1, crossover_rate = 1,
             number_of_individuals=0):
        pass
