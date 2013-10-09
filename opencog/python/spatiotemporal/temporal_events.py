from datetime import datetime

from scipy.stats import t

from spatiotemporal.interval import Interval
from spatiotemporal.time import is_unix_time, UnixTime


__author__ = 'keyvan'


class BaseTemporalEvent(object):
    def __init__(self, a, b, iter_step=1):
        self.interval = Interval(a, b, iter_step)

    def pdf_single_point(self, time_step):
        """
        to override, default pdf calls to it
        alternatively one can directly override pdf
        """
        if self.interval.a <= time_step <= self.interval.b:
            return 1
        return 0

    def pdf(self, x):
        """
        time can either be a single value or a collection
        """
        if type(x) is TemporalEventPiecewise:
            pass

        result = []
        try:
            for time_step in x:
                result.append(self.pdf_single_point(time_step))
        except:
            assert is_unix_time(x), "'x' should be unix time when 'pdf' is invoked by a singular argument"
            return self.pdf_single_point(x)
        return result

    def _interval_from_self_if_none(self, a, b, interval):
        if interval is None:
            if (a, b) == (None, None):
                interval = self.interval
            else:
                interval = Interval(a, b)
        else:
            assert hasattr(interval, 'a') and hasattr(interval, 'b') and is_unix_time(
                interval.a) and is_unix_time(interval.b) and interval.a < interval.b, "passed 'interval' should have " \
                                                                                      "'a' and 'b' attributes, both " \
                                                                                      "of which should be in unix " \
                                                                                      "time, and 'b' has to be " \
                                                                                      "greater than 'a'"
        return interval

    def probability_in_interval(self, a=None, b=None, interval=None):
        """
        use either 'a' and 'b' or 'interval'
        """
        #TODO replace by CalculateCenterOfMass
        interval = self._interval_from_self_if_none(a, b, interval)
        return max(self.pdf(interval))

    @property
    def a(self):
        return self.interval.a

    @a.setter
    def a(self, value):
        assert is_unix_time(value), "'a' should be unix time"
        self.interval.a = value

    @property
    def b(self):
        return self.interval.b

    @b.setter
    def b(self, value):
        assert is_unix_time(value), "'b' should be unix time"
        self.interval.b = value

    @property
    def iter_step(self):
        return self.interval.iter_step

    @iter_step.setter
    def iter_step(self, value):
        self.interval.iter_step = value

    def x_axis(self):
        return self.interval.x_axis()

    def __iter__(self):
        return iter(self.interval)

    def __len__(self):
        return len(self.interval)

    def __repr__(self):
        return 'spatiotemporal.temporal_events.{0}(a:{1}, b:{2})'.format(
            self.__class__.__name__,  self.interval.a, self.interval.b)

    def __str__(self):
        return repr(self)


class TemporalEventSimple(BaseTemporalEvent):
    pass


class TemporalEventDistributional(BaseTemporalEvent):
    def __init__(self, a, b, pdf, iter_step=1):
        BaseTemporalEvent.__init__(self, a, b, iter_step=iter_step)
        assert callable(pdf), "'pdf' should be callable"
        self.pdf = pdf
        self.pdf_single_point = pdf


def _interpolation_a_to_beginning(t, a, beginning):
    _a = 1 / (beginning - a)
    _b = -1 * _a * a
    return _a * t + _b


def _interpolation_ending_to_b(t, ending, b):
    _a = -1 / (b - ending)
    _b = -1 * _a * b
    return _a * t + _b


def _interpolation_a_to_b(t, a, beginning, ending, b):
    t = UnixTime(t)
    if t <= a or t >= b:
        return 0
    if a < t < beginning:
        return _interpolation_a_to_beginning(t, a, beginning)
    if ending < t < b:
        return _interpolation_ending_to_b(t, ending, b)
    return 1


class TemporalEventPiecewise(BaseTemporalEvent):
    beginning_factor = 5
    ending_factor = 5
    _beginning = -1
    _ending = -1

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
            while not self.interval.a < self._beginning < self._ending < self.interval.b:
                self._beginning = self.interval.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.interval.a + len(self) / self.beginning_factor,
                        len(self) / self.beginning_factor
                    )
                )

                self._ending = self.interval.random_time(
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        self.interval.b - len(self) / self.ending_factor,
                        len(self) / self.ending_factor
                    )
                )

    def pdf_single_point(self, time_step):
        return _interpolation_a_to_b(time_step, self.a, self.beginning, self.ending, self.b)

    def probability_in_interval(self, a=None, b=None, interval=None):
        interval = self._interval_from_self_if_none(a, b, interval)
        if self.a <= interval.a <= self.beginning and self.ending <= interval.b <= self.b:
            return 1
        return max(self.pdf_single_point(interval.a), self.pdf_single_point(interval.b))

    @property
    def beginning(self):
        return self._beginning

    @beginning.setter
    def beginning(self, value):
        if self._ending < 0:
            assert self.interval.a < value < self.interval.b, "'beginning' should be within ['a' : 'b'] interval"
        else:
            assert self.interval.a < value < self._ending, "'beginning' should be within ['a' : 'ending'] interval"
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
        self._ending = value

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
        print event.probability_in_interval()

    return events


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    events = generate_random_events(1)

    for event in events:
        y = event.pdf(event)
        plt.plot(event.x_axis(), y)

    plt.show()