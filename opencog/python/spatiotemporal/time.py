__author__ = 'keyvan'
from calendar import timegm
from datetime import datetime
from random import randrange
from scipy.stats import rv_continuous, norm, t, uniform
import numpy as np
import matplotlib.pyplot as plt


def convert_to_unix_time(time):
    if time is None:
        return None
    if isinstance(time, datetime):
        return timegm(time.utctimetuple())
    assert isinstance(time, int)
    return time


def random_time(start, end=None, probability_distribution=None):
    """
    start and end can be in either datetime or unix time
    return value is in unix time
    """
    start = convert_to_unix_time(start)
    end = convert_to_unix_time(end)
    if probability_distribution is None:
        return randrange(start, end)
    assert isinstance(probability_distribution, rv_continuous)
    return probability_distribution.rvs(xrange(start, end))


class Period(object):
    def __init__(self, start, end):
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
        return self.end - self.start


class TemporalEvent(object):
    beginning_factor = 5
    ending_factor = 5

    def __init__(self, start, end, **kwargs):
        """
        start and end are datetime
        """
        self.period = Period(start, end)
        if 'beginning_factor' in kwargs:
            self.beginning_factor = kwargs['beginning_factor']
        if 'ending_factor' in kwargs:
            self.ending_factor = kwargs['ending_factor']
        self.length_beginning = 8

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
    dist = norm
    n = 500
    a = [x for x in xrange(n)]
    b = dist.rvs(size=n)
    c = dist.pdf(a, 250, 50)
    print a, b

    plt.plot(a, b)
    #plt.plot(a, c)
    plt.show()

    #print dist.rvs(2)
    #generate_random_events()

#uniform_2010 = [1 for x in year_2010]
#print len(year_2010)
#
##dist = uniform()
#h = plt.plot(year_2010, norm.pdf(year_2010))
##
#plt.show()
#
##x = 'abcde'
##y1 = uniform.pdf(x)
###y1 = t.pdf(x, 1, '3')
###y2 = norm.pdf(x, '3')
##
##plt.plot(x, y1)
###plt.plot(x, y2)
##plt.xlabel('T')
##plt.ylabel('P')
##plt.show()