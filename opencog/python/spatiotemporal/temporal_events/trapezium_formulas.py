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
    #from numpy import convolve
    #from scipy.signal import fftconvolve
    _1_9 = 1.0/9
    _1_4 = 1.0/4
    #print fftconvolve([_1_9, _1_9, _1_9, 0], [0, _1_4, _1_4, _1_4])
    #
    #f = function_convolution_uniform((1, 10), (7, 11))
    #print 'precede:', f.integrate(MINUS_INFINITY, -0.25)
    #print 'meet:', f.integrate(-0.25, 0.25)
    #print 'overlap:', f.integrate(0.25, PLUS_INFINITY)
    #
    #print 'precede:', f.integrate(MINUS_INFINITY, -2)
    #print 'meet:', f.integrate(-2, 2)
    #print 'overlap:', f.integrate(2, PLUS_INFINITY)
    #
    #print f.integrate(MINUS_INFINITY, PLUS_INFINITY)
    #f.plot().show()

    from scipy import signal
    from scipy import ndimage
    import numpy as np

    np.random.seed(1)

    for N in range(1, 10):
        #a = np.random.randint(0, 10, size=(N, N))
        #b = np.random.randint(0, 10, size=(N, N))
        a, b = range(N), range(N)

        r1 = np.convolve(a, b)

        print len(r1)