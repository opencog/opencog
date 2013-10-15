from datetime import datetime
from scipy.stats import t
from spatiotemporal.interval import Interval, assert_is_interval
from spatiotemporal.membership_function import FuzzyMembershipFunction
from spatiotemporal.unix_time import UnixTime
from utility.geometric import LinearFunction

__author__ = 'keyvan'


class BaseTemporalEvent(Interval):
    def __init__(self, a, b, iter_step=1):
        Interval.__init__(self, a, b, iter_step=iter_step)
        self.membership_function = FuzzyMembershipFunction(self, self.membership_function_single_point)

    def membership_function_single_point(self, time_step):
        return 0

    def _interval_from_self_if_none(self, a, b, interval):
        if interval is None:
            if (a, b) == (None, None):
                interval = self
            else:
                interval = Interval(a, b)
        else:
            assert_is_interval(interval)
        return interval

    def probability_in_interval(self, a=None, b=None, interval=None):
        """
        use either 'a' and 'b' or 'interval'
        """
        interval = self._interval_from_self_if_none(a, b, interval)
        return max(self.membership_function(interval))

    def __repr__(self):
        return 'spatiotemporal.temporal_events.{0}(a:{1}, b:{2})'.format(
            self.__class__.__name__, self.interval.a, self.interval.b)

    def __str__(self):
        return repr(self)


class TemporalEventSimple(BaseTemporalEvent):
    def membership_function_single_point(self, time_step):
        if self.a <= time_step <= self.b:
            return 1
        return 0


class TemporalEventDistributional(BaseTemporalEvent):
    def __init__(self, a, b, pdf, iter_step=1):
        assert callable(pdf), "'pdf' should be callable"
        BaseTemporalEvent.__init__(self, a, b, iter_step=iter_step)
        self.pdf_single_point = pdf


class TemporalEventPiecewise(BaseTemporalEvent):
    beginning_factor = 5
    ending_factor = 5
    _beginning = -1
    _ending = -1
    _linear_function_a_to_beginning = None
    _linear_function_ending_to_b = None

    def __init__(self, a, b, beginning=None, ending=None, beginning_factor=None, ending_factor=None, iter_step=1):
        """
        start and end can be in either datetime or unix time
        """
        if (beginning, ending) != (None, None):
            assert (beginning_factor, ending_factor) == (None, None), "PiecewiseTemporalEvent() only accepts " \
                                                                      "either 'beginning_factor' and 'ending_factor' " \
                                                                      "or 'beginning' and 'ending'"
        BaseTemporalEvent.__init__(self, a, b, iter_step=iter_step)

        if beginning_factor is not None:
            assert beginning_factor > 0
            self.beginning_factor = beginning_factor
        if ending_factor is not None:
            assert ending_factor > 0
            self.ending_factor = ending_factor

        if (beginning, ending) != (None, None):
            self.beginning = beginning
            self.ending = ending
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
        if time_step <= self.a or time_step >= self.b:
            return 0
        if self.a < time_step < self.beginning:
            return self.linear_function_a_to_beginning(time_step)
        if self.ending < time_step < self.b:
            return self.linear_function_ending_to_b(time_step)
        return 1

    def probability_in_interval(self, a=None, b=None, interval=None):
        interval = self._interval_from_self_if_none(a, b, interval)
        if self.a <= interval.a <= self.beginning and self.ending <= interval.b <= self.b:
            return 1
        return max(self.membership_function_single_point(interval.a), self.membership_function_single_point(interval.b))

    @Interval.a.setter
    def a(self, value):
        self._linear_function_a_to_beginning = None
        Interval.a.fset(self, value)

    @Interval.b.setter
    def b(self, value):
        self._linear_function_ending_to_b = None
        Interval.b.fset(self, value)

    @property
    def beginning(self):
        return self._beginning

    @beginning.setter
    def beginning(self, value):
        if self._ending < 0:
            assert self.interval.a < value < self.interval.b, "'beginning' should be within ['a' : 'b'] interval"
        else:
            assert self.interval.a < value < self._ending, "'beginning' should be within ['a' : 'ending'] interval"
        self._linear_function_a_to_beginning = None
        self._beginning = value

    @property
    def ending(self):
        return self._ending

    @ending.setter
    def ending(self, value):
        if self._beginning < 0:
            assert self.interval.a < value < self.interval.b, "'ending' should be within ['a' : 'b'] interval"
        else:
            assert self._beginning < value < self.interval.b, "'ending' should be within ['beginning' : 'b'] interval"
        self._linear_function_ending_to_b = None
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

    def __repr__(self):
        return 'spatiotemporal.temporal_events.PiecewiseTemporalEvent(a:{0} , beginning:{1}, ending:{2}, b:{3})'.format(
            self.interval.a, self.beginning, self.ending, self.interval.b)


def generate_random_events(size=20):
    events = []

    year_2010 = Interval(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(size):
        start = year_2010.random_time()
        end = year_2010.random_time(start)
        event = TemporalEventPiecewise(start, end, iter_step=100)
        events.append(event)

    return events


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import time

    #event = TemporalEventSimple(datetime(2010, 1, 1), datetime(2011, 2, 1), iter_step=100)
    #event = plt.plot(event.to_list(), event.membership_function())
    #plt.show()

    events = generate_random_events(5)

    start = time.time()

    for event in events:
        plt.plot(event.to_list(), event.membership_function())

    list_performance = time.time() - start

    plt.show()
    #plt.clf()
    #
    #start = time.time()
    #
    #for event in events:
    #    plt.plot(event, event.pdf)
    #
    #print list_performance, 'vs', time.time() - start
    #
    #plt.show()