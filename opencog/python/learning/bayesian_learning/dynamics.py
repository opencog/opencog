from decimal import Decimal
from random import random as rand
from learning.bayesian_learning.network import Row
from sample_data.uci_adult_dataset import main

__author__ = 'keyvan'


def _givens_and_negations_from(kwargs):
    if kwargs.has_key('givens'):
        givens = set(kwargs['givens'])
    else:
        givens=set()
    if kwargs.has_key('negations'):
        negations = set(kwargs['negations'])
    else:
        negations=set()
    return givens, negations

class DataObserver(list):
    """
    Container with the ability of calculating probabilities
    In each time-step, only add the set of the names of the variables
    that had value of True in that time-step (all the other variables
    are considered false for this step, open-world assumption)
    """

    epsilon = 0.00001

    def probability_of(self, variable, value=True, **kwargs):
        """
        Calculates probability for value of True or False by
        having given and negation parameters, based on perceived
        data.
        e.g P(A|B,C) : probability_of('A', givens=['B','C'])
        P(~A|B,~C) : probability_of('A', False, givens=['B'], negations=['C'])
        """
        givens, negations = _givens_and_negations_from(kwargs)
        matched, total = 0, Decimal(0)
        for record in self:
            if givens.issubset(record) and len(negations & record) == 0:
                total += 1
                if record[variable] is value:
                    matched += 1
        if not total:
            return total
        return matched / total


TEST_VARIABLES = ['A','B','C','D','E']
A,B,C,D,E = TEST_VARIABLES

def generate_test_record():
    """
    Bayes net: A->C , B->C, B->D, E is independent
    By current probability distribution:
    P(C) = 0.46 = Marginalising over all possible configurations
    C and D are 'conditionally independent' given B
    """
    record = Row()
    if rand() < 0.5: # P(A) = 0.5
        record.add(A)
    if rand() < 0.6: # P(B) = 0.6
        record.add(B)

    if record == set([A,B]):
        if rand() < 0.9: # P(C|A,B) = 0.9
            record.add(C)
    elif record == set(A):
        if rand() < 0.4: # P(C|A,~B) = 0.4
            record.add(C)
    elif record == set(B):
        if rand() < 0.3: # P(C|~A,B) = 0.3
            record.add(C)
    else:
        if rand() < 0.1: # P(C|~A,~B) = 0.1
            record.add(C)

    if B in record:
        if rand() < 0.6: # P(D|B) = 0.6
            record.add(D)

    if rand() < 0.4:
        record.add(E) # P(E) = 0.4, independent of the rest

    return record

def generate_test_dataset(dataset_size):
    data = DataObserver()
    for i in range(dataset_size):
        data.append(main.uci_adult())
    return data
    #print data.probability_of('C', givens=[A], negations=[B])
    #    print data.probability_of('D', givens=['A','B'])
