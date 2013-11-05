from math import fabs
from numpy import isinf
from utility.numeric.globals import EPSILON

__author__ = 'keyvan'

DISTRIBUTION_INTEGRAL_LIMIT = 1 - EPSILON


def calculate_bounds_of_probability_distribution(probability_distribution,
                                                 distribution_integral_limit=DISTRIBUTION_INTEGRAL_LIMIT):
    a, b = probability_distribution.interval(1)
    if isinf(a) or isinf(b):
        a, b = probability_distribution.interval(distribution_integral_limit)
    return a, b