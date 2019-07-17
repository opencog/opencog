__author__ = 'Keyvan'

from math import log

def mutual_information(seq, first_variable, second_variable, epsilon=0.0001):
    M = float(len(seq))

    n_first, n_second = {True:0, False:0}, {True:0, False:0}

    n_first_and_second = {(True,True):0, (True,False):0,
                          (False,True):0, (False,False):0}
    for record in seq:
        value_of_first, value_of_second = first_variable in record,\
                                          second_variable in record
        n_first[value_of_first] += 1
        n_second[value_of_second] += 1
        n_first_and_second[(value_of_first, value_of_second)] += 1

    for dict in n_first, n_second, n_first_and_second:
        for key in dict:
            if dict[key] is 0:
                dict[key] = epsilon


    result = 0
    for key in n_first_and_second:
        value_of_first, value_of_second = key
        result += n_first_and_second[key] / M * log(n_first_and_second[key] *\
                                                    M / n_first[value_of_first] / n_second[value_of_second])
    return result
