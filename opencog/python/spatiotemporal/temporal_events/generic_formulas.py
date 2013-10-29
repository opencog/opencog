from math import fabs
from numpy import linspace, convolve
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear

__author__ = 'keyvan'

def function_convolution_generic(pdf_1, bounds_1, pdf_2, bounds_2, bins=50):
    a1, b1 = bounds_1
    a2, b2 = bounds_2
    length_1 = fabs(a1 - b1)
    length_2 = fabs(a2 - b2)

    convolution_bounds_a, convolution_bounds_b = min(a1, a2), max(b1, b2)

    delta = fabs(convolution_bounds_a - convolution_bounds_b) / bins
    range = linspace(convolution_bounds_a, convolution_bounds_b, bins)
    x = [pdf_1(t) for t in range]
    y = [pdf_2(t) for t in range]

    c = convolve(x, y)
    #c = c[::-1]

    index_a, index_b = 0, 0
    for i in xrange(len(c) - 1):
        if not c[i] > 0:
            index_a += 1
        else:
            break
    for i in xrange(len(c) - 1, -1, -1):
        if not c[i] > 0:
            index_b += 1
        else:
            break
    index_a -= 1
    index_b = len(c) - index_b

    transform_a = (float(b1 - a2) - (a1 - b2)) / (index_b - index_a)
    transform_b = float(a1 - b2) - index_a * transform_a

    transform_function = lambda x: transform_a * x + transform_b
    transform_function_inverse = lambda y: int((y - transform_b) / transform_a)

    convolution_function = FunctionPiecewiseLinear([transform_function(t) for t in xrange(len(c))],
                                                   c, FunctionHorizontalLinear(0))
    return convolution_function.normalised()

if __name__ == '__main__':
    from scipy.stats import norm, uniform
    from spatiotemporal.time_intervals import TimeInterval
    from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY

    dist_1 = norm(loc=5.5, scale=4.5)
    dist_2 = norm(loc=9, scale=2)
    #dist_1 = uniform(loc=1, scale=9)
    #dist_2 = uniform(loc=7, scale=4)

    bounds_1 = 1, 10
    bounds_2 = 7, 11

    fn = function_convolution_generic(dist_1.pdf, bounds_1, dist_2.pdf, bounds_2)
    print fn.integrate(-15, 4)
    #plt = fn.plot().show()

    times_overlapped = float(0)
    times_preceded = float(0)
    times_met = float(0)
    size = 100000
    prec = 0.25
    x = []

    for i in xrange(size):
        rv1 = dist_1.rvs()
        if rv1 <= 0:
            rv1 = 0.01
        rv2 = dist_2.rvs()
        if rv2 >= 12:
            rv2 = 11.99
        instance_1 = TimeInterval(0, rv1)
        instance_2 = TimeInterval(rv2, 12)
        d = instance_1.b - instance_2.a

        if fabs(d) <= prec:
            times_met += 1
        elif instance_1.b < instance_2.a:
            times_preceded += 1
        else:
            times_overlapped += 1
        x.append(d)

    o = times_overlapped / size
    p = times_preceded / size
    m = times_met / size
    print 'o:', o
    print 'p:', p
    print 'm:', m

    print 'precede:', fn.integrate(MINUS_INFINITY, -prec)
    print 'meet:', fn.integrate(-prec, prec)
    print 'overlap:', fn.integrate(prec, PLUS_INFINITY)

    plt = fn.plot()
    plt.ylim(ymax=0.5)
    plt.hist(x, 50, normed=True)
    x1 = linspace(*bounds_1)
    x2 = linspace(*bounds_2)
    plt.plot(x1, dist_1.pdf(x1))
    plt.plot(x2, dist_2.pdf(x2))
    plt.show()