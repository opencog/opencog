__author__ = 'keyvan'
from calendar import timegm
from datetime import datetime
from random import randrange
from scipy.stats import t
from scipy.stats.distributions import rv_frozen

TEMPORAL_RELATIONS = {
    'p': 'precedes',
    'm': 'meets',
    'o': 'overlaps',
    'F': 'finished by',
    'D': 'contains',
    's': 'starts',
    'e': 'equals',
    'S': 'started by',
    'd': 'during',
    'f': 'finishes',
    'O': 'overlapped by',
    'M': 'met by',
    'P': 'preceded by'
}


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
    assert isinstance(probability_distribution, rv_frozen)
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

    def __init__(self, start, end, beginning_factor=None, ending_factor=None):
        """
        start and end can be in either datetime or unix time
        """
        self.period = Period(start, end)

        if beginning_factor is not None:
            assert beginning_factor > 0
            self.beginning_factor = beginning_factor
        if ending_factor is not None:
            assert ending_factor > 0
            self.ending_factor = ending_factor

        self.beginning, self.ending = 0, 0

        while not self.period.start < self.beginning < self.ending < self.period.end:
            self.beginning = self.period.random(
                probability_distribution=t(
                    # df, mean, variance
                    4,
                    self.period.start + len(self) / self.beginning_factor,
                    len(self) / self.beginning_factor
                )
            )

            self.ending = self.period.random(
                probability_distribution=t(
                    # df, mean, variance
                    4,
                    self.period.end - len(self) / self.ending_factor,
                    len(self) / self.ending_factor
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
        """
        time can either be a single value or a collection
        """
        result = []
        try:
            for time_step in time:
                result.append(self._fuzzy_membership_single_point(time_step))
        except:
            return self._fuzzy_membership_single_point(time)
        return result

    def __str__(self):
        return 'TemporalEvent(start:{0} , beginning:{1}, ending:{2}, end:{3})'.format(self.period.start,
                                                                                      self.beginning,
                                                                                      self.ending,
                                                                                      self.period.end)

    def __repr__(self):
        return str(self)

    def __len__(self):
        return len(self.period)


class TemporalRelation(list):
    def __init__(self, constituent_string, temporal_event_1, temporal_event_2):
        self.append(constituent_string)
        assert isinstance(temporal_event_1, TemporalEvent) and isinstance(temporal_event_2, TemporalEvent)
        self.temporal_event_1 = temporal_event_1
        self.temporal_event_2 = temporal_event_2

    def append(self, other):
        for char in other:
            assert isinstance(char, str) and len(char) is 1 and char in TEMPORAL_RELATIONS
            list.append(self, char)

    def degree(self):
        return 0.5

    def __eq__(self, other):
        assert isinstance(other, TemporalRelation) or isinstance(other, str)
        if isinstance(other, TemporalRelation):
            if (other.temporal_event_1, other.temporal_event_2) != (self.temporal_event_1, self.temporal_event_2):
                return False
        for char in other:
            if char not in self:
                return False
        return True

    def __str__(self):
        if len(self) == len(TEMPORAL_RELATIONS):
            return 'TemporalRelation(pmoFDseSdfOMP(full))'
        if self == 'oFDseSdfO':
            return 'TemporalRelation(oFDseSdfO(concur))'
        return 'TemporalRelation({0}({1}))'.format(
            ''.join(self),
            ', '.join([TEMPORAL_RELATIONS[char] for char in self]))

    def __repr__(self):
        return str(self)


def generate_random_events(size=100):
    events = []
    year_2010 = Period(datetime(2010, 1, 1), datetime(2011, 1, 1))

    for i in xrange(size):
        start = year_2010.random()
        end = year_2010.random(start)
        event = TemporalEvent(start, end)
        events.append(event)

    return events


def create_event_relation_hashtable(temporal_events):
    table = {}

    for A in temporal_events:
        for B in temporal_events:
            if B == A:
                continue
            for C in temporal_events:
                if C in (A, B):
                    continue
                for r1 in TEMPORAL_RELATIONS:
                    for r2 in TEMPORAL_RELATIONS:
                        for r3 in TEMPORAL_RELATIONS:
                            relation1 = TemporalRelation(r1, A, B)
                            relation2 = TemporalRelation(r2, B, C)
                            relation3 = TemporalRelation(r3, A, C)
                            degrees = (relation1.degree(), relation2.degree(), relation3.degree())
                            if degrees != (0, 0, 0):
                                table[(r1, r2, r3)] = degrees

    return table


if __name__ == '__main__':
    #events = generate_random_events()
    #import matplotlib.pyplot as plt
    #
    #for event in events:
    #    x = range(event.period.start, event.period.end, 100)
    #    y = event.fuzzy_membership(x)
    #    plt.plot(x, y)
    #
    #plt.show()

    events = generate_random_events(10)
    table = create_event_relation_hashtable(events)
    print table