from math import fabs
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear
from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY

__author__ = 'keyvan'


def function_convolution_uniform(bounds_1, bounds_2):
    a1, b1 = bounds_1
    a2, b2 = bounds_2
    length_1 = fabs(a1 - b1)
    length_2 = fabs(a2 - b2)

    convolution_bounds_a, convolution_bounds_b = a1 - b2, b1 - a2

    trapezium_0, trapezium_1 = convolution_bounds_a, convolution_bounds_a + min(length_2, length_1)
    trapezium_2, trapezium_3 = trapezium_1 + fabs(length_1 - length_2), convolution_bounds_b
    assert trapezium_2 + min(length_2, length_1) == trapezium_3

    p = min(1 / length_1, 1 / length_2)


    convolution_function = FunctionPiecewiseLinear([trapezium_0, trapezium_1, trapezium_2, trapezium_3],
                                                   [0, p, p, 0], FunctionHorizontalLinear(0))
    return convolution_function


def temporal_relations_between(trapezium_event_1, trapezium_event_2):
    pass

if __name__ == '__main__':
    #f = function_convolution_uniform((1, 10), (7, 11))
    #plt = f.plot()
    #plt.show()
    ##plt.clf()
    #f = function_convolution_uniform((1, 3), (7, 10))
    f = function_convolution_uniform((7, 11), (1, 10))
    print 'precede:', f.integrate(MINUS_INFINITY, -0.25)
    print 'meet:', f.integrate(-0.25, 0.25)
    print 'overlap:', f.integrate(0.25, PLUS_INFINITY)
    print f.integrate(MINUS_INFINITY, PLUS_INFINITY)
    f.plot().show()