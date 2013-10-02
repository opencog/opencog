__author__ = 'keyvan'
from calendar import timegm
from datetime import datetime
from random import randrange
from scipy.stats import t #, rv_frozen
import matplotlib.pyplot as plt


def is_unix_time(time):
    return (isinstance(time, int) or isinstance(time, float)) and time > 0


def convert_to_unix_time(time):
    if time is None:
        return None
    if isinstance(time, datetime):
        return timegm(time.utctimetuple())
    assert is_unix_time(time)
    return time


def random_time(start, end=None,
                probability_distribution=None):
    """
    start and end can be in either datetime or unix time
    return value is in unix time
    """
    assert start is not None
    start = convert_to_unix_time(start)
    end = convert_to_unix_time(end)
    if probability_distribution is None:
        return randrange(start, end)
        #assert isinstance(probability_distribution, rv_frozen)
    assert end is not None
    random_value = probability_distribution.rvs()
    if random_value < start:
        return start
    if random_value > end:
        return end
    return random_value


class Period(object):
    def __init__(self, start, end):
        """
        start and end can be in either datetime or unix time
        """
        self.start = convert_to_unix_time(start)
        self.end = convert_to_unix_time(end)
        assert self.start <= self.end

    def random(self, start=None, end=None,
               probability_distribution=None):
        if start is None:
            start = self.start
        else:
            start = convert_to_unix_time(start)
            assert start >= self.start
        if end is None:
            end = self.end
        else:
            end = convert_to_unix_time(end)
            assert end <= self.end
        return random_time(start, end, probability_distribution)

    def __iter__(self):
        return xrange(self.start, self.end)

    def __len__(self):
        return float(self.end - self.start)


class TemporalEvent(object):
    beginning_factor = 5
    ending_factor = 5

    def __init__(self, start, end, **kwargs):
        """
        start and end can be in either datetime or unix time
        """
        self.period = Period(start, end)

        if 'beginning_factor' in kwargs:
            self.beginning_factor = kwargs['beginning_factor']
        if 'ending_factor' in kwargs:
            self.ending_factor = kwargs['ending_factor']

        self.beginning = self.period.random(
            probability_distribution=t(
                4, # df
                self.period.start + len(self) / self.beginning_factor, # mean
                len(self) / self.beginning_factor                       # variance
            )
        )

        self.ending = self.period.random(
            probability_distribution=t(
                4, # df
                self.period.end - len(self) / self.ending_factor, # mean
                len(self) / self.ending_factor                          # variance
            )
        )

    def _fuzzy_membership_single_point(self, time_step):
        assert is_unix_time(time_step)
        if time_step <= self.period.start or time_step >= self.period.end:
            return 0
        if self.period.start < time_step < self.beginning:
            a = 1 / (self.beginning - self.period.start)
            b = -1 * a * self.period.start
            return a * time_step + b
        if self.ending < time_step < self.period.end:
            a = -1 / (self.period.end - self.ending)
            b = -1 * a * self.period.end
            return a * time_step + b
        return 1

    def fuzzy_membership(self, time):
        result = []
        try:
            for time_step in time:
                result.append(self._fuzzy_membership_single_point(time_step))
        except:
            return self._fuzzy_membership_single_point(time)
        return result

    def __len__(self):
        return len(self.period)


def generate_random_events():
    events = []
    year_2010 = Period(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(10):
        start = year_2010.random()
        end = year_2010.random(start)
        event = TemporalEvent(start, end)
        events.append(event)

    return events


if __name__ == '__main__':
    #print dist.rvs(2)
    events = generate_random_events()

    for event in events:
        x = range(event.period.start, event.period.end, 100)
        y = event.fuzzy_membership(x)
        plt.plot(x, y)

    plt.show()