from math import fabs
from numpy import convolve
from scipy.stats.distributions import uniform_gen
from spatiotemporal.temporal_events.trapezium_formulas import function_convolution_uniform
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.temporal_interval_handling import calculateCenterMass
from spatiotemporal.time_intervals import TimeInterval
from utility.geometric import FunctionPiecewiseLinear, FunctionHorizontalLinear

__author__ = 'keyvan'


def function_convolution_generic(dist_1, dist_2, bins=50):
    a1, b1 = calculate_bounds_of_probability_distribution(dist_1)
    a2, b2 = calculate_bounds_of_probability_distribution(dist_2)

    if (type(dist_1.dist), type(dist_2.dist)) is (uniform_gen, uniform_gen):
        return function_convolution_uniform((a1, b1), (a2, b2))

    convolution_bounds_a, convolution_bounds_b = min(a1, a2), max(b1, b2)

    delta = fabs(convolution_bounds_a - convolution_bounds_b) / bins
    convolution_interval = TimeInterval(convolution_bounds_a, convolution_bounds_b, bins)
    x = [dist_1.pdf(t) for t in convolution_interval]
    y = [dist_2.pdf(t) for t in reversed(convolution_interval)]

    c = convolve(x, y)
    dictionary_convolution = {}
    for t in xrange(len(c)):
        dictionary_convolution[delta * t] = c[t]
    bias = calculateCenterMass(dictionary_convolution)[0] + dist_2.mean() - dist_1.mean()
    dictionary_convolution_biased = {}
    for t in dictionary_convolution:
        dictionary_convolution_biased[t - bias] = dictionary_convolution[t]

    convolution_function = FunctionPiecewiseLinear(dictionary_convolution_biased, FunctionHorizontalLinear(0))
    return convolution_function.normalised()


def temporal_relations_between(temporal_event_1, temporal_event_2, prec):
    beginning_a_beginning_b = temporal_event_1.distribution_beginning, temporal_event_2.distribution_ending
    beginning_a_ending_b = temporal_event_1.distribution_beginning, temporal_event_2.distribution_ending
    ending_a_beginning_b = temporal_event_1.distribution_ending, temporal_event_2.distribution_beginning
    ending_a_ending_b = temporal_event_1.distribution_ending, temporal_event_2.distribution_ending

    function_beginning_a_beginning_b = function_convolution_generic(*beginning_a_beginning_b)
    function_beginning_a_ending_b = function_convolution_generic(*beginning_a_ending_b)
    function_ending_a_beginning_b = function_convolution_generic(*ending_a_beginning_b)
    function_ending_a_ending_b = function_convolution_generic(*ending_a_ending_b)

    before = NEGATIVE_INFINITY, -prec
    same = -prec, prec
    after = prec, POSITIVE_INFINITY

    convolutions = {
        beginning_a_beginning_b: (
            function_beginning_a_beginning_b.integral(*before),
            function_beginning_a_beginning_b.integral(*same),
            function_beginning_a_beginning_b.integral(*after)
        ),
        beginning_a_ending_b: (
            function_beginning_a_ending_b.integral(*before),
            function_beginning_a_ending_b.integral(*same),
            function_beginning_a_ending_b.integral(*after)
        ),
        ending_a_beginning_b: (
            function_ending_a_beginning_b.integral(*before),
            function_ending_a_beginning_b.integral(*same),
            function_ending_a_beginning_b.integral(*after)
        ),
        ending_a_ending_b: (
            function_ending_a_ending_b.integral(*before),
            function_ending_a_ending_b.integral(*same),
            function_ending_a_ending_b.integral(*after)
        ),
    }

    before = 0
    same = 1
    after = 2

    result = {
        'p':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][before] * convolutions[ending_a_ending_b][before],

        'm':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][same] * convolutions[ending_a_ending_b][before],

        'o':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'F':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'D':
        convolutions[beginning_a_beginning_b][before] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        's':
        convolutions[beginning_a_beginning_b][same] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'e':
        convolutions[beginning_a_beginning_b][same] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'S':
        convolutions[beginning_a_beginning_b][same] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        'd':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][before],

        'f':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][same],

        'O':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][before] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        'M':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][same] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],

        'P':
        convolutions[beginning_a_beginning_b][after] * convolutions[beginning_a_ending_b][after] *
        convolutions[ending_a_beginning_b][after] * convolutions[ending_a_ending_b][after],
    }

    # Adjusting the integral error
    summed_up = sum(result.values())
    print summed_up
    for predicate in result:
        result[predicate] /= summed_up
    return result


def test(event_1, event_2, prec=0.25, size=100000):
    import matplotlib.pyplot as plt

    times_overlapped = float(0)
    times_preceded = float(0)
    times_met = float(0)
    x = []

    for i in xrange(size):
        instance_1 = event_1.instance()
        instance_2 = event_2.instance()
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
    print 'after:', o
    print 'same:', m
    print 'before:', p

    plt.hist(x, 50, normed=True)


if __name__ == '__main__':
    from scipy.stats import norm, uniform, expon
    from spatiotemporal.temporal_events import TemporalEvent, TemporalEventPiecewiseLinear
    from numpy import NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY
    import matplotlib.pyplot as plt

    prec = 0.25

    for event_1, event_2 in [
        (
            TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4)),
            TemporalEvent(uniform(loc=8, scale=5), uniform(loc=15, scale=4))
        ),

        (
            TemporalEvent(uniform(loc=0, scale=2), uniform(loc=6, scale=4)),
            TemporalEvent(uniform(loc=3, scale=2), uniform(loc=13, scale=4))
        ),

        (
            TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4)),
            TemporalEvent(uniform(loc=0, scale=11), uniform(loc=13, scale=4))
        ),

        (
            TemporalEvent(uniform(loc=4, scale=1), uniform(loc=10, scale=2)),
            TemporalEvent(uniform(loc=2, scale=4), uniform(loc=7, scale=7))
        ),

        (
            TemporalEvent(norm(loc=1, scale=4.5), expon(loc=30, scale=2)),
            TemporalEvent(norm(loc=25, scale=4.5), expon(loc=60, scale=2))
        ),

        (
            TemporalEvent(expon(loc=1, scale=4.5), norm(loc=30, scale=2)),
            TemporalEvent(expon(loc=25, scale=4.5), norm(loc=60, scale=2))
        ),

        (
            TemporalEventPiecewiseLinear({1: 0, 2: 0.1, 3: 0.3, 4: 0.7, 5: 1}, {6: 1, 7: 0.9, 8: 0.6, 9: 0.1, 10: 0}),
            TemporalEventPiecewiseLinear({7.5: 0, 8.5: 0.1, 9.5: 0.3, 10.5: 0.7, 11.5: 1},
                                         {13: 1, 14.5: 0.9, 15.3: 0.6, 17: 0.1, 20: 0})
        ),
    ]:
        #plt = event_1.distribution_ending.plot()
        #plt.ylim(ymin=-0, ymax=0.6)
        #plt.show()
        #break

        relations = temporal_relations_between(event_1, event_2, prec)
        for predicate in 'PMOfdSesDFomp':
            if predicate == 'o':
                print '-------------------------'
            print predicate, ':', relations[predicate]
            if predicate == 'p':
                print '-------------------------'

        event_1.plot().ylim(ymin=-0.1, ymax=1.1)
        plt = event_2.plot()
        plt.figure()
        fn = function_convolution_generic(event_1.distribution_ending, event_2.distribution_beginning)
        before = NEGATIVE_INFINITY, -prec
        same = -prec, prec
        after = prec, POSITIVE_INFINITY
        fn.integral(*before),
        fn.integral(*same),
        fn.integral(*after)
        fn.plot()
        test(event_1, event_2, prec)
        plt.show()
