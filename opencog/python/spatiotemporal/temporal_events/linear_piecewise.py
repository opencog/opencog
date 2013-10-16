from scipy.stats import t
from spatiotemporal.interval import Interval
from spatiotemporal.temporal_events.generic import BaseTemporalEventWithCustomListImplementation
from spatiotemporal.unix_time import UnixTime
from utility.geometric import HorizontalLinearFunction, CompositeFunction, LinearFunction
from utility.numeric.globals import MINUS_INFINITY, PLUS_INFINITY, EPSILON

__author__ = 'keyvan'


class TemporalEventLinearPiecewise(BaseTemporalEventWithCustomListImplementation):
    beginning_factor = 5
    ending_factor = 5
    _beginning = -1
    _ending = -1
    _linear_function_a_to_beginning = None
    _linear_function_ending_to_b = None
    linear_function_beginning_to_ending = HorizontalLinearFunction(1)
    linear_function_minus_inf_to_a = HorizontalLinearFunction(0)
    linear_function_b_to_plus_inf = linear_function_minus_inf_to_a
    _composite_function = None

    def __init__(self, a, b, beginning=None, ending=None, beginning_factor=None, ending_factor=None):
        """
        start and end can be in either datetime or unix time
        """
        if (beginning, ending) != (None, None):
            assert (beginning_factor, ending_factor) == (None, None), "PiecewiseTemporalEvent() only accepts " \
                                                                      "either 'beginning_factor' and 'ending_factor' " \
                                                                      "or 'beginning' and 'ending'"
        BaseTemporalEventWithCustomListImplementation.__init__(self, a, b)

        if beginning_factor is not None:
            assert beginning_factor > 0
            self.beginning_factor = beginning_factor
        if ending_factor is not None:
            assert ending_factor > 0
            self.ending_factor = ending_factor

        if (beginning, ending) != (None, None):
            self.beginning = UnixTime(beginning)
            self.ending = UnixTime(ending)
        else:
            while not self.a < self._beginning < self._ending < self.b:
                self._beginning = self.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.a + self.duration / self.beginning_factor,
                        self.duration / self.beginning_factor
                    )
                )

                self._ending = self.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.b - self.duration / self.ending_factor,
                        self.duration / self.ending_factor
                    )
                )

    def membership_function_single_point(self, time_step):
        time_step = UnixTime(time_step)
        return self.composite_function(time_step)

    def degree_in_interval(self, a=None, b=None, interval=None):
        interval = self._interval_from_self_if_none(a, b, interval)
        return self.composite_function.integrate(interval.a, interval.b) / (interval.b - interval.a)

    def temporal_relation_with(self, other):
        pass

    def _update_composite_function(self):
        self._composite_function = CompositeFunction(
            {
                (MINUS_INFINITY, self.a): self.linear_function_minus_inf_to_a,
                (self.a, self.beginning): self.linear_function_a_to_beginning,
                (self.beginning, self.ending): self.linear_function_beginning_to_ending,
                (self.ending, self.b): self.linear_function_ending_to_b,
                (self.b, PLUS_INFINITY): self.linear_function_b_to_plus_inf
            }
        )

    def to_list(self):
        return [self.a, self.a + EPSILON, self.beginning, self.ending, self.b - EPSILON, self.b]

    @Interval.a.setter
    def a(self, value):
        self._linear_function_a_to_beginning = None
        self._composite_function = None
        Interval.a.fset(self, value)

    @Interval.b.setter
    def b(self, value):
        self._linear_function_ending_to_b = None
        self._composite_function = None
        Interval.b.fset(self, value)

    @property
    def beginning(self):
        return self._beginning

    @beginning.setter
    def beginning(self, value):
        if self._ending < 0:
            assert self.a < value < self.b, "'beginning' should be within ['a' : 'b'] interval"
        else:
            assert self.a < value < self._ending, "'beginning' should be within ['a' : 'ending'] interval"
        self._linear_function_a_to_beginning = None
        self._composite_function = None
        self._beginning = value

    @property
    def ending(self):
        return self._ending

    @ending.setter
    def ending(self, value):
        if self._beginning < 0:
            assert self.a < value < self.b, "'ending' should be within ['a' : 'b'] interval"
        else:
            assert self._beginning < value < self.b, "'ending' should be within ['beginning' : 'b'] interval"
        self._linear_function_ending_to_b = None
        self._composite_function = None
        self._ending = value

    @property
    def linear_function_a_to_beginning(self):
        if self._linear_function_a_to_beginning is None:
            _a = 1 / (self.beginning - self.a)
            _b = -1 * _a * self.a
            self._linear_function_a_to_beginning = LinearFunction(_a, _b)
        return self._linear_function_a_to_beginning

    @property
    def linear_function_ending_to_b(self):
        if self._linear_function_ending_to_b is None:
            _a = -1 / (self.b - self.ending)
            _b = -1 * _a * self.b
            self._linear_function_ending_to_b = LinearFunction(_a, _b)
        return self._linear_function_ending_to_b

    @property
    def composite_function(self):
        if self._composite_function is None:
            self._update_composite_function()
        return self._composite_function

    def __repr__(self):
        return 'PiecewiseTemporalEvent(a: {0} , beginning: {1}, ending: {2}, b: {3})'.format(
            self.a, self.beginning, self.ending, self.b)


def generate_random_events(size=20):
    from datetime import datetime
    events = []

    year_2010 = Interval(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(size):
        start = year_2010.random_time()
        end = year_2010.random_time(start)
        event = TemporalEventLinearPiecewise(start, end)
        events.append(event)

    return events


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import time

    #------------------
    #events = generate_random_events(800)
    events = generate_random_events(5)

    start = time.time()

    for event in events:
        plt.plot(event, event.membership_function)

    print 'Performance:', time.time() - start, 'seconds'

    plt.show()

