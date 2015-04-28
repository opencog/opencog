__author__ = 'keyvan'

from utility.evolutionary import *

BIT_STRING_LENGTH = 32
ALL_BITS_ONE = 2**BIT_STRING_LENGTH - 1

class BitStringIndividualBase(IndividualBase):

    def __init__(self, _binary_chromosome=0):
        self.chromosome = _binary_chromosome

    @staticmethod
    def randomly_initialised():
        instance = BitStringIndividualBase()
        instance.chromosome = randrange(0,ALL_BITS_ONE)
        return instance

    loci = range(BIT_STRING_LENGTH)

    def __fitness__(self):
        result = 0
        for i in range(BIT_STRING_LENGTH):
            if self[i] == 1:
                result += 1
        return result

    def __getitem__(self, index):
        if self.chromosome & 2**index == 0:
            return 0
        return 1

    def __setitem__(self, index, value):
        if value == 1:
            self.chromosome |=  2**index
        else:
            self.chromosome &=  ALL_BITS_ONE ^ 2**index

    def __mutate__(self):
        mask = 2**randrange(0, BIT_STRING_LENGTH)
        return BitStringIndividualBase(self.chromosome ^ mask)

    def __repr__(self):
        result = ''
        for i in range(BIT_STRING_LENGTH):
            result = str(self[i]) + result
        return result

if __name__ == '__main__':
    population = []
    for i in range(32):
        population.append(BitStringIndividualBase.randomly_initialised())
    ga = GeneticAlgorithm(population)
    for i in range(10):
        fittest = ga.step(0, 1)
        print fittest, fittest.fitness
    print 'Solution found:'
    print ga.fittest_individual_found, ga.highest_fitness_found