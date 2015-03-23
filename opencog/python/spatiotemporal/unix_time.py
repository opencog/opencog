from datetime import datetime
from time import mktime
from random import random
from scipy.stats.distributions import rv_frozen

__author__ = 'keyvan'


class UnixTime(float):
    def __new__(cls, datetime_or_number):
        if type(datetime_or_number) is UnixTime:
            return datetime_or_number
        assert isinstance(
            datetime_or_number, (datetime, int, float, long)), "UnixTime() argument must be a datetime or a number, " \
                                                               "not '{0}'".format(datetime_or_number.__class__.__name__)
        if isinstance(datetime_or_number, datetime):
            fraction = datetime_or_number.microsecond / float(10 ** 6)
            value = mktime(datetime_or_number.timetuple())
            value += fraction
        else:
            value = datetime_or_number

        return float.__new__(cls, value)

    def to_datetime(self):
        return datetime.fromtimestamp(self)

    def __repr__(self):
        return 'UnixTime({0}: {1})'.format(float(self), str(self))

    def __str__(self):
        return str(self.to_datetime())


def is_unix_time(time):
    return isinstance(time, (UnixTime, int, float, long)) and time >= 0


def random_time(start, stop, probability_distribution=None):
    """
    start and stop can be in either datetime or unix time
    return value is in UnixTime
    """
    start = UnixTime(start)
    stop = UnixTime(stop)
    assert start < stop, "'stop' should be greater than start"
    if probability_distribution is None:
        return UnixTime(random() * (stop - start) + start)
    assert isinstance(probability_distribution,
                      rv_frozen), "'probability_distribution' should be a scipy frozen distribution"
    random_value = probability_distribution.rvs()
    if random_value < start:
        return start
    if random_value > stop:
        return stop
    return UnixTime(random_value)

