from scipy.stats import t
from spatiotemporal.temporal_events.generic import TemporalEventPiecewiseLinear, TemporalInstance
from spatiotemporal.unix_time import UnixTime, random_time

__author__ = 'keyvan'


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
        TemporalEventPiecewiseLinear.__init__(self, {a: 0, beginning: 1}, {ending: 1, b: 0})


def generate_random_events(size=20):
    from datetime import datetime
    from spatiotemporal.time_intervals import TimeInterval
    events = []

    year_2010 = TimeInterval(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(size):
        start = year_2010.random_time()
        end = year_2010.random_time(start)
        event = TemporalEventTrapezium(start, end)
        events.append(event)

    return events


if __name__ == '__main__':
    import time

    #event = TemporalEventTrapezium(1, 20)
    #
    #event.plot()
    #event.distribution_beginning.plot()
    #plt = event.distribution_ending.plot()
    #plt.ylim(ymin=0, ymax=1.1)
    #plt.show()

    events = generate_random_events(5)
    #events = generate_random_events(500)

    start = time.time()

    for event in events:
        plt = event.plot()

    print 'Performance:', time.time() - start, 'seconds'

    plt.ylim(ymin=0, ymax=1.1)
    plt.show()
