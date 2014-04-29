from math import fabs, sqrt
from numpy import isinf
from scipy.stats import expon
from scipy.stats.distributions import uniform_gen
from utility.numeric.globals import EPSILON

__author__ = 'keyvan'

DISTRIBUTION_INTEGRAL_LIMIT = 1 - EPSILON


def calculate_bounds_of_probability_distribution(probability_distribution,
                                                 distribution_integral_limit=DISTRIBUTION_INTEGRAL_LIMIT):
    if isinstance(probability_distribution.dist, uniform_gen):
        mean, var = probability_distribution.mean(), probability_distribution.var()
        length = sqrt(12 * var)
        a = mean - length / 2.0
        return a, a + length

    a, b = probability_distribution.interval(1)
    a_is_inf, b_is_inf = isinf(a), isinf(b)
    if a_is_inf or b_is_inf:
        a_alternative, b_alternative = probability_distribution.interval(distribution_integral_limit)
        if a_is_inf:
            a = a_alternative
        if b_is_inf:
            b = b_alternative
    return a, b

if __name__ == '__main__':

    print calculate_bounds_of_probability_distribution(expon(loc=10, scale=2))