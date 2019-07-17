from math import fabs, sqrt, floor
from numpy import convolve, NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY
import numpy
from scipy.stats.distributions import uniform_gen
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.temporal_interval_handling import calculateCenterMass
from spatiotemporal.time_intervals import TimeInterval
from utility.functions import FunctionPiecewiseLinear, FunctionHorizontalLinear, integral, FUNCTION_ZERO, almost_equals


DECOMPOSITION_PRECISION = 10 ** 14


__author__ = 'keyvan'

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


class TemporalRelation(dict):
    all_relations = 'pmoFDseSdfOMP'
    _type = None
    _list = None
    _vector = None

    @staticmethod
    def from_list(list_object):
        relation = TemporalRelation()
        for i, name in enumerate(TemporalRelation.all_relations):
            value = list_object[i]
            if not isinstance(value, (int, float)):
                value = float(value)
            relation[name] = value
        return relation

    def to_list(self):
        if self._list is None:
            self._list = []
            for name in self.all_relations:
                self._list.append(self[name])
        return self._list

    def to_vector(self):
        if self._vector is None:
            _list = self.to_list()
            self._vector = numpy.array(_list)
        return self._vector

    @property
    def type(self):
        if self._type is None:
            self._type = ''.join([name for name in TemporalRelation.all_relations if self[name] > 0])
        return self._type

    def __setitem__(self, relation_name, value):
        if relation_name not in TemporalRelation.all_relations:
            raise AttributeError("'{0}' is not a valid Allen relation'".format(relation_name))
        dict.__setitem__(self, relation_name, floor(value * DECOMPOSITION_PRECISION) / DECOMPOSITION_PRECISION)

    def __repr__(self):
        return 'TemporalRelation({0})'.format(self.type)

    def __str__(self):
        return repr(self)

    def __hash__(self):
        return hash(tuple(self.to_list()))


class BaseRelationFormula(object):
    def __init__(self):
        self.bounds = {}

    def duration_of(self, dist):
        a, b = self.bounds_of(dist)
        return fabs(a - b)

    def bounds_of(self, dist):
        # if dist in self.bounds:
        #     return self.bounds[dist]

        bounds = calculate_bounds_of_probability_distribution(dist)
        self.bounds[dist] = bounds
        return bounds

    def before_point(self, point_1_value, point_2_value):
        return 0

    def same_point(self, point_1_value, point_2_value):
        return 1 - fabs(self.before_point(point_1_value,
                                          point_2_value) - self.after_point(point_1_value, point_2_value))

    def after_point(self, point_1_value, point_2_value):
        return self.before_point(point_2_value, point_1_value)

    def before_integral_bounds(self, dist_1, dist_2):
        return calculate_bounds_of_probability_distribution(dist_1)

    def same_integral_bounds(self, dist_1, dist_2):
        dist_1_a, dist_1_b = calculate_bounds_of_probability_distribution(dist_1)
        dist_2_a, dist_2_b = calculate_bounds_of_probability_distribution(dist_2)
        return max(dist_1_a, dist_2_a), min(dist_1_b, dist_2_b)

    def after_integral_bounds(self, dist_1, dist_2):
        return calculate_bounds_of_probability_distribution(dist_2)

    def before(self, dist_1, dist_2):
        return integral(lambda x: self.before_point(dist_1.pdf(x), dist_2.pdf(x)),
                        *self.before_integral_bounds(dist_1, dist_2))

    def same(self, dist_1, dist_2):
        return integral(lambda x: self.same_point(dist_1.pdf(x), dist_2.pdf(x)),
                        *self.same_integral_bounds(dist_1, dist_2))

    def after(self, dist_1, dist_2):
        return integral(lambda x: self.after_point(dist_1.pdf(x), dist_2.pdf(x)),
                        *self.after_integral_bounds(dist_1, dist_2))

    def compare(self, dist_1, dist_2):
        """
        returns before, same and after
        """
        return self.before(dist_1, dist_2), self.same(dist_1, dist_2), self.after(dist_1, dist_2)


class FormulaCreator(object):
    def __init__(self, relation_formula):
        self.relation_formula = relation_formula

    def temporal_relations_between(self, temporal_event_1, temporal_event_2):
        dist_1_beginning, dist_1_ending = temporal_event_1.distribution_beginning, temporal_event_1.distribution_ending
        dist_2_beginning, dist_2_ending = temporal_event_2.distribution_beginning, temporal_event_2.distribution_ending
        self.relation_formula.bounds[dist_1_beginning] = temporal_event_1.a, temporal_event_1.beginning
        self.relation_formula.bounds[dist_1_ending] = temporal_event_1.ending, temporal_event_1.b
        self.relation_formula.bounds[dist_2_beginning] = temporal_event_2.a, temporal_event_2.beginning
        self.relation_formula.bounds[dist_2_ending] = temporal_event_2.ending, temporal_event_2.b

        combinations = [
            (dist_1_beginning, dist_2_beginning),
            (dist_1_beginning, dist_2_ending),
            (dist_1_ending, dist_2_beginning),
            (dist_1_ending, dist_2_ending)
        ]

        return self.calculate_relations(combinations)

    def calculate_relations(self, combinations=None):
        """
        Calculates the values of the 13 relations based on the before, same,
        and after values of the combinations between the beginning and
        ending distributions of the two intervals obtained, e.g. from
        the DecompositionFitter.
        :param combinations: the 4 combinations between beginning and ending
        distribution
        :return: a dictionary containing the 13 relations as keys and their
        degrees as values
        """
        if combinations is None:
            combinations = self.relation_formula.combinations

        dist_1_beginning, dist_2_beginning = combinations[0]
        dist_1_ending, dist_2_ending = combinations[3]

        before = {}
        same = {}
        after = {}
        # iterates over the 4 combinations between beginning and ending
        for key in combinations:
            before[key], same[key], after[key] = self.relation_formula.compare(*key)

        result = TemporalRelation()

        result['p'] = before[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      before[dist_1_ending, dist_2_beginning] * before[dist_1_ending, dist_2_ending]

        result['m'] = before[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      same[dist_1_ending, dist_2_beginning] * before[dist_1_ending, dist_2_ending]

        result['o'] = before[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * before[dist_1_ending, dist_2_ending]

        result['F'] = before[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * same[dist_1_ending, dist_2_ending]

        result['D'] = before[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * after[dist_1_ending, dist_2_ending]

        result['s'] = same[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * before[dist_1_ending, dist_2_ending]

        result['e'] = same[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * same[dist_1_ending, dist_2_ending]

        result['S'] = same[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * after[dist_1_ending, dist_2_ending]

        result['d'] = after[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * before[dist_1_ending, dist_2_ending]

        result['f'] = after[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * same[dist_1_ending, dist_2_ending]

        result['O'] = after[dist_1_beginning, dist_2_beginning] * before[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * after[dist_1_ending, dist_2_ending]

        result['M'] = after[dist_1_beginning, dist_2_beginning] * same[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * after[dist_1_ending, dist_2_ending]

        result['P'] = after[dist_1_beginning, dist_2_beginning] * after[dist_1_beginning, dist_2_ending] * \
                      after[dist_1_ending, dist_2_beginning] * after[dist_1_ending, dist_2_ending]

        return result


class RelationFormulaConvolution(BaseRelationFormula):
    def function_convolution_uniform(self, bounds_1, bounds_2, probability=None):
        a1, b1 = bounds_1
        a2, b2 = bounds_2
        length_1 = fabs(a1 - b1)
        length_2 = fabs(a2 - b2)

        convolution_bounds_a, convolution_bounds_b = a1 - b2, b1 - a2

        trapezium_0, trapezium_1 = convolution_bounds_a, convolution_bounds_a + min(length_2, length_1)
        trapezium_2, trapezium_3 = trapezium_1 + fabs(length_1 - length_2), convolution_bounds_b
        #assert trapezium_2 + min(length_2, length_1) == trapezium_3

        if probability is None:
            probability = min(1 / length_1, 1 / length_2)

        result = FunctionPiecewiseLinear(
            {trapezium_0: 0, trapezium_1: probability, trapezium_2: probability, trapezium_3: 0},
            FUNCTION_ZERO)
        result.is_normalised = True

        return result

    def function_convolution(self, dist_1, dist_2, bins=50):
        a_1, b_1, a_2, b_2 = 0, 0, 0, 0
        if dist_1 in self.bounds:
            a_1, b_1 = self.bounds[dist_1]
        else:
            a_1, b_1 = calculate_bounds_of_probability_distribution(dist_1)
            self.bounds[dist_1] = a_1, b_1
        if dist_2 in self.bounds:
            a_2, b_2 = self.bounds[dist_2]
        else:
            a_2, b_2 = calculate_bounds_of_probability_distribution(dist_2)
            self.bounds[dist_2] = a_2, b_2

        if (type(dist_1.dist), type(dist_2.dist)) == (uniform_gen, uniform_gen):
            return self.function_convolution_uniform((a_1, b_1), (a_2, b_2))

        convolution_bounds_a, convolution_bounds_b = min(a_1, a_2), max(b_1, b_2)

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

    def calculate_similarity(self, dist_1, dist_2):
        if (type(dist_1.dist), type(dist_2.dist)) == (uniform_gen, uniform_gen):
            length_dist_1 = self.duration_of(dist_1)
            length_dist_2 = self.duration_of(dist_2)
            return min(length_dist_1, length_dist_2) / sqrt(length_dist_1 * length_dist_2)

        dist_1_mean, dist_2_mean = dist_1.mean(), dist_2.mean()

        dist_1_transformed = lambda t: dist_1.pdf(t + dist_1_mean)
        dist_2_transformed = lambda t: dist_2.pdf(t + dist_2_mean)

        geometric_mean = lambda t: sqrt(dist_1_transformed(t) * dist_2_transformed(t))
        return integral(geometric_mean, NEGATIVE_INFINITY, POSITIVE_INFINITY)

    def compare(self, dist_1, dist_2):
        convolution = self.function_convolution(dist_1, dist_2)
        before = integral(convolution, NEGATIVE_INFINITY, 0)
        after = integral(convolution, 0, POSITIVE_INFINITY)
        similarity = self.calculate_similarity(dist_1, dist_2)
        correlation = 1 - fabs(before - after)
        same = similarity * correlation

        return before, same, after


class RelationFormulaGeometricMean(BaseRelationFormula):
    def compare(self, dist_1, dist_2):
        dist_1_interval = TimeInterval(*self.bounds_of(dist_1))
        dist_2_interval = TimeInterval(*self.bounds_of(dist_2))
        dictionary_input_output = {}
        for time_step in dist_1_interval + dist_2_interval:
            dictionary_input_output[time_step] = sqrt(dist_1.pdf(time_step) * dist_2.pdf(time_step))

        geometric_mean = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=FUNCTION_ZERO)
        same = integral(geometric_mean, NEGATIVE_INFINITY, POSITIVE_INFINITY)

        dist_1_mean, dist_1_skewness, dist_1_kurtosis = dist_1.stats(moments='msk')
        dist_1_standard_deviation = dist_1.std()
        dist_2_mean, dist_2_skewness, dist_2_kurtosis = dist_2.stats(moments='msk')
        dist_2_standard_deviation = dist_2.std()

        distance = fabs(dist_1_standard_deviation - dist_2_standard_deviation) + fabs(dist_1_skewness - dist_2_skewness)
        distance += fabs(dist_1_kurtosis - dist_2_kurtosis)
        delta = dist_1_mean - dist_2_mean
        non_same_portion = 1.0 - same

        portion_after, portion_before = 1.0, 0.0
        if almost_equals(distance, 0):
            if delta < 0:
                portion_after, portion_before = 0.0, 1.0
        else:
            dist_1_standardized_pdf = lambda x: dist_1.pdf(dist_1_standard_deviation * x + dist_1_mean)
            dist_2_standardized_pdf = lambda x: dist_2.pdf(dist_2_standard_deviation * x + dist_2_mean)

            geometric_mean = lambda t: sqrt(dist_1_standardized_pdf(t) * dist_2_standardized_pdf(t))
            geometric_mean_scaled = lambda p: geometric_mean(p / distance)
            geometric_mean_scaled_length = max(self.duration_of(dist_1), self.duration_of(dist_2))

            dictionary_input_output = {}
            for time_step in TimeInterval(-geometric_mean_scaled_length / 2.0, geometric_mean_scaled_length / 2.0):
                dictionary_input_output[time_step] = geometric_mean_scaled(time_step)

            geometric_mean_scaled = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=FUNCTION_ZERO)
            portion_after = integral(geometric_mean_scaled, NEGATIVE_INFINITY, delta)
            portion_before = integral(geometric_mean_scaled, delta, POSITIVE_INFINITY)

        after = portion_after / (portion_after + portion_before) * non_same_portion
        return 1.0 - same - after, same, after


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from scipy.stats import norm, uniform, expon
    from spatiotemporal.temporal_events import TemporalEvent, TemporalEventPiecewiseLinear
    import matplotlib.pyplot as plt

    figure_number = 1
    for event_1, event_2 in [
        (
            TemporalEvent(uniform(loc=3, scale=2), uniform(loc=7, scale=9)),
            TemporalEvent(uniform(loc=0, scale=10), uniform(loc=13, scale=2))
        ),
        #
        # (
        #     TemporalEvent(uniform(loc=0, scale=2), uniform(loc=3, scale=2)),
        #     TemporalEvent(uniform(loc=3, scale=2), uniform(loc=6, scale=2))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4)),
        #     TemporalEvent(uniform(loc=8, scale=5), uniform(loc=15, scale=4))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=0, scale=2), uniform(loc=6, scale=4)),
        #     TemporalEvent(uniform(loc=3, scale=2), uniform(loc=13, scale=4))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=0, scale=7), uniform(loc=8, scale=7)),
        #     TemporalEvent(uniform(loc=4, scale=1), uniform(loc=11, scale=2)),
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4)),
        #     TemporalEvent(uniform(loc=0, scale=11), uniform(loc=13, scale=4))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=1, scale=8), uniform(loc=6, scale=8)),
        #     TemporalEvent(uniform(loc=0, scale=22), uniform(loc=13, scale=8))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=2, scale=2), uniform(loc=7, scale=2)),
        #     TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=1, scale=2), uniform(loc=4, scale=2)),
        #     TemporalEvent(uniform(loc=6, scale=2), uniform(loc=9, scale=2))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=0, scale=3), uniform(loc=15, scale=2)),
        #     TemporalEvent(uniform(loc=5, scale=2), uniform(loc=9, scale=3))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=5, scale=3), uniform(loc=9, scale=2)),
        #     TemporalEvent(uniform(loc=1, scale=2), uniform(loc=15, scale=3))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=0, scale=2), uniform(loc=10, scale=2)),
        #     TemporalEvent(uniform(loc=15, scale=2), uniform(loc=25, scale=2))
        # ),
        #
        # (
        #     TemporalEvent(uniform(loc=15, scale=2), uniform(loc=25, scale=2)),
        #     TemporalEvent(uniform(loc=0, scale=2), uniform(loc=10, scale=2))
        # ),
        #
        # (
        #     TemporalEvent(norm(loc=1, scale=4.5), expon(loc=30, scale=2)),
        #     TemporalEvent(norm(loc=25, scale=4.5), expon(loc=60, scale=2))
        # ),
        #
        # (
        #     TemporalEvent(expon(loc=1, scale=4.5), norm(loc=30, scale=2)),
        #     TemporalEvent(expon(loc=25, scale=4.5), norm(loc=60, scale=2))
        # ),
        #
        # (
        #     TemporalEventPiecewiseLinear({1: 0, 2: 0.1, 3: 0.3, 4: 0.7, 5: 1}, {6: 1, 7: 0.9, 8: 0.6, 9: 0.1, 10: 0}),
        #     TemporalEventPiecewiseLinear({7.5: 0, 8.5: 0.1, 9.5: 0.3, 10.5: 0.7, 11.5: 1},
        #                                  {13: 1, 14.5: 0.9, 15.3: 0.6, 17: 0.1, 20: 0})
        # ),

    ]:

        temporal_relations = event_1 * event_2

        print '\nFigure' + str(figure_number)
        print '----------------------'
        print sum(temporal_relations.values())
        for p in 'pmoFDseSdfOMP':
            print p, temporal_relations[p]

        figure_number += 1

        event_1.plot(show_distributions=True).ylim(ymin=-0.1, ymax=1.1)
        event_2.plot(show_distributions=True).figure()

    plt.show()
