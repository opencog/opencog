from math import fabs
from utility.geometric import FunctionPiecewiseLinear, FUNCTION_ZERO

__author__ = 'keyvan'


def function_convolution_uniform(bounds_1, bounds_2):
    a1, b1 = bounds_1
    a2, b2 = bounds_2
    length_1 = fabs(a1 - b1)
    length_2 = fabs(a2 - b2)

    convolution_bounds_a, convolution_bounds_b = a1 - b2, b1 - a2

    trapezium_0, trapezium_1 = convolution_bounds_a, convolution_bounds_a + min(length_2, length_1)
    trapezium_2, trapezium_3 = trapezium_1 + fabs(length_1 - length_2), convolution_bounds_b
    #assert trapezium_2 + min(length_2, length_1) == trapezium_3

    p = min(1 / length_1, 1 / length_2)

    result = FunctionPiecewiseLinear({trapezium_0: 0, trapezium_1: p, trapezium_2: p, trapezium_3: 0}, FUNCTION_ZERO)
    result.is_normalised = True
    return result
