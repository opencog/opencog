from scipy.stats import t
from spatiotemporal.time_intervals import TimeIntervalIterBased
from spatiotemporal.temporal_events.generic import TemporalEventPiecewiseLinear
from spatiotemporal.unix_time import UnixTime, random_time

__author__ = 'keyvan'

EPSILON = 0.00001


class TemporalEventTrapezium(TemporalEventPiecewiseLinear):
    beginning_factor = 5
    ending_factor = 5

    def __init__(self, a, b, beginning=None, ending=None, beginning_factor=None, ending_factor=None):
        """
        start and end can be in either datetime or unix time
        """
        a, b = UnixTime(a), UnixTime(b)
        assert a < b, "'b' should be greater than 'a'"
        if (beginning, ending) != (None, None):
            assert (beginning_factor, ending_factor) == (None, None), "PiecewiseTemporalEvent() only accepts " \
                                                                      "either 'beginning_factor' and 'ending_factor' " \
                                                                      "or 'beginning' and 'ending'"

        if beginning_factor is not None:
            assert beginning_factor > 0
            self.beginning_factor = beginning_factor
        if ending_factor is not None:
            assert ending_factor > 0
            self.ending_factor = ending_factor

        if (beginning, ending) != (None, None):
            beginning = UnixTime(beginning)
            ending = UnixTime(ending)
        else:
            beginning, ending = 0, 0
            while not a < beginning < ending < b:
                beginning = random_time(
                    a,
                    b,
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        a + float(b - a) / self.beginning_factor,
                        float(b - a) / self.beginning_factor
                    )
                )

                ending = random_time(
                    a,
                    b,
                    probability_distribution=t(
                        # df, mean, variance
                        4,
                        b - float(b - a) / self.ending_factor,
                        float(b - a) / self.ending_factor
                    )
                )
        TemporalEventPiecewiseLinear.__init__(self, [a, beginning, ending, b], [0, 1, 1, 0])
        self.insert(1, UnixTime(a + EPSILON))
        self.insert(4, UnixTime(b - EPSILON))
        self.membership_function_single_point.output_list.insert(1, self.membership_function_single_point(self[1]))
        self.membership_function_single_point.output_list.insert(4, self.membership_function_single_point(self[4]))
        self.membership_function_single_point.invalidate()

    @TemporalEventPiecewiseLinear.a.setter
    def a(self, value):
        assert value > self[1]
        self[0] = UnixTime(value)
        self[1] = UnixTime(value + EPSILON)
        self.membership_function_single_point.invalidate()

    @TemporalEventPiecewiseLinear.b.setter
    def b(self, value):
        assert value > self[-2]
        self[-1] = UnixTime(value)
        self[-2] = UnixTime(value - EPSILON)
        self.membership_function_single_point.invalidate()

    @property
    def beginning(self):
        return self[2]

    @beginning.setter
    def beginning(self, value):
        assert self.a < value < self.ending, "'beginning' should be within ['a' : 'ending'] interval"
        self[2] = value
        self.membership_function_single_point.invalidate()

    @property
    def ending(self):
        return self[-3]

    @ending.setter
    def ending(self, value):
        assert self._beginning < value < self.b, "'ending' should be within ['beginning' : 'b'] interval"
        self[-3] = value
        self.membership_function_single_point.invalidate()

    def __repr__(self):
        return 'PiecewiseTemporalEvent(a: {0} , beginning: {1}, ending: {2}, b: {3})'.format(
            self.a, self.beginning, self.ending, self.b)


def generate_random_events(size=20):
    from datetime import datetime
    events = []

    year_2010 = TimeIntervalIterBased(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(size):
        start = year_2010.random_time()
        end = year_2010.random_time(start)
        event = TemporalEventTrapezium(start, end)
        events.append(event)

    return events


if __name__ == '__main__':
    import time

    #events = generate_random_events(1000)
    events = generate_random_events(5)

    start = time.time()

    for event in events:
        plt = event.plot()

    print 'Performance:', time.time() - start, 'seconds'

    plt.show()
