from math import fabs, sqrt
from numpy import convolve, NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY
from scipy.stats.distributions import uniform_gen
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.temporal_interval_handling import calculateCenterMass
from spatiotemporal.time_intervals import TimeInterval
from utility.functions import FunctionPiecewiseLinear, FunctionHorizontalLinear, integral, FUNCTION_ZERO, almost_equals
from utility.numeric import EPSILON


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

    def to_list(self):
        result = []
        for name in self.all_relations:
            result.append(self[name])
        return result

    def __repr__(self):
        return 'TemporalRelation({0})'.format(''.join([name for name in self.all_relations if self[name] > 0]))

    def __str__(self):
        return repr(self)


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
        # if not isinstance(relation_formula, (BaseRelationFormula)):
        #     raise TypeError("'relation_formula' should be an instance of BaseTemporalFormula or Individual")
        self.relation_formula = relation_formula

    def temporal_relations_between(self, temporal_event_1, temporal_event_2):
        beginning_a, ending_a = temporal_event_1.distribution_beginning, temporal_event_1.distribution_ending
        beginning_b, ending_b = temporal_event_2.distribution_beginning, temporal_event_2.distribution_ending
        self.relation_formula.bounds[beginning_a] = temporal_event_1.a, temporal_event_1.beginning
        self.relation_formula.bounds[ending_a] = temporal_event_1.ending, temporal_event_1.b
        self.relation_formula.bounds[beginning_b] = temporal_event_2.a, temporal_event_2.beginning
        self.relation_formula.bounds[ending_b] = temporal_event_2.ending, temporal_event_2.b

        combinations = [
            (beginning_a, beginning_b),
            (beginning_a, ending_b),
            (ending_a, beginning_b),
            (ending_a, ending_b)
        ]

        return self.calculate_relations(combinations)

    def calculate_relations(self, combinations=None):
        if combinations is None:
            combinations = self.relation_formula.combinations

        beginning_a, beginning_b = combinations[0]
        ending_a, ending_b = combinations[3]

        before = {}
        same = {}
        after = {}
        for key in combinations:
            before[key], same[key], after[key] = self.relation_formula.compare(*key)

        result = TemporalRelation()

        result['p'] = before[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      before[ending_a, beginning_b] * before[ending_a, ending_b]

        result['m'] = before[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      same[ending_a, beginning_b] * before[ending_a, ending_b]

        result['o'] = before[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * before[ending_a, ending_b]

        result['F'] = before[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * same[ending_a, ending_b]

        result['D'] = before[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * after[ending_a, ending_b]

        result['s'] = same[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * before[ending_a, ending_b]

        result['e'] = same[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * same[ending_a, ending_b]

        result['S'] = same[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * after[ending_a, ending_b]

        result['d'] = after[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * before[ending_a, ending_b]

        result['f'] = after[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * same[ending_a, ending_b]

        result['O'] = after[beginning_a, beginning_b] * before[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * after[ending_a, ending_b]

        result['M'] = after[beginning_a, beginning_b] * same[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * after[ending_a, ending_b]

        result['P'] = after[beginning_a, beginning_b] * after[beginning_a, ending_b] * \
                      after[ending_a, beginning_b] * after[ending_a, ending_b]

        print [(before[m, n], same[m, n], after[m, n]) for m in [beginning_a, ending_a] for n in [beginning_b, ending_b]]

        return result


class RelationFormulaGM(BaseRelationFormula):
    def before_point(self, point_1_value, point_2_value):
        return point_1_value ** 2

    def same_point(self, point_1_value, point_2_value):
        return sqrt(point_1_value * point_2_value)

    def before(self, dist_1, dist_2):
        return integral(lambda x: self.before_point(dist_1.pdf(x), 0) - self.same_point(dist_1.pdf(x), dist_2.pdf(x)),
                        *self.before_integral_bounds(dist_1, dist_2))

    def after(self, dist_1, dist_2):
        return self.before(dist_2, dist_1)


class RelationFormulaSimple(BaseRelationFormula):
    def before(self, dist_1, dist_2):
        a, b = self.bounds_of(dist_1)
        # Cancel pdf's error
        a += EPSILON
        b -= EPSILON
        interval = TimeInterval(a, b)
        dictionary_input_output = {}
        for time_step in interval:
            dictionary_input_output[time_step] = dist_1.pdf(time_step) * (1 - dist_2.cdf(time_step))
        function = FunctionPiecewiseLinear(dictionary_input_output, FUNCTION_ZERO)
        return integral(function, a, b)

    def after(self, dist_1, dist_2):
        return self.before(dist_2, dist_1)

    # def before(self, dist_1, dist_2):
    #     dist_1_a, dist_1_b = self.bounds_of(dist_1)
    #     dist_2_a, dist_2_b = self.bounds_of(dist_2)
    #     a, b = dist_1_a, dist_2_a
    #     interval = TimeInterval(a, b, 200)
    #     dictionary_input_output = {}
    #     for time_step in interval:
    #         dictionary_input_output[time_step] = dist_1.pdf(time_step) ** 2
    #     function = FunctionPiecewiseLinear(dictionary_input_output, FUNCTION_ZERO)
    #     return integral(function, a, b)

    def same(self, dist_1, dist_2):
        dist_1_a, dist_1_b = self.bounds_of(dist_1)
        dist_2_a, dist_2_b = self.bounds_of(dist_2)
        a, b = max(dist_1_a, dist_2_a), min(dist_1_b, dist_2_b)
        interval = TimeInterval(a, b)
        dictionary_input_output = {}
        for time_step in interval:
            dictionary_input_output[time_step] = dist_1.pdf(time_step) * dist_2.pdf(time_step)
        function = FunctionPiecewiseLinear(dictionary_input_output, FUNCTION_ZERO)
        return integral(function, a, b)

    # def compare(self, dist_1, dist_2):
    #     same = self.same(dist_1, dist_2)
    #     non_same = 1 - same
    #     before = self.before(dist_1, dist_2)
    #     after = self.before(dist_2, dist_1)
    #     before = before / (before + after) * non_same
    #     after = after / (before + after) * non_same
    #     return before, same, after


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
        raise NotImplementedError()

    def compare(self, dist_1, dist_2):
        convolution = self.function_convolution(dist_1, dist_2)
        a_1, b_1 = self.bounds_of(dist_1)
        a_2, b_2 = self.bounds_of(dist_2)
        before = integral(convolution, NEGATIVE_INFINITY, 0)
        after = integral(convolution, 0, POSITIVE_INFINITY)
        similarity = self.calculate_similarity(dist_1, dist_2)
        correlation = 1 - fabs(before - after)
        same = similarity * correlation

        return before, same, after

    # def compare(self, dist_1, dist_2):
    #     convolution = self.function_convolution(dist_1, dist_2)
    #     geometric_mean = lambda x: min(dist_1.pdf(x), dist_2.pdf(x))
    #     first_moment = lambda x: x * geometric_mean(x)
    #
    #     integral_geometric_mean = integral(geometric_mean, NEGATIVE_INFINITY, POSITIVE_INFINITY)
    #     integral_first_moment = integral(first_moment, NEGATIVE_INFINITY, POSITIVE_INFINITY)
    #
    #     if integral_geometric_mean > 0:
    #         # print 'shodes', integral_geometric_mean
    #         mean_of_geometric_mean = integral_first_moment / integral_geometric_mean
    #         transformed_geometric_mean = lambda x: geometric_mean(x / 2.0 - mean_of_geometric_mean)
    #     else:
    #         # print geometric_mean(20.5)
    #         transformed_geometric_mean = lambda x: 0
    #
    #     same = integral(lambda x: min(convolution(x), transformed_geometric_mean(x)), NEGATIVE_INFINITY,
    #                     POSITIVE_INFINITY)
    #
    #     convolution_before_same = lambda x: convolution(x) - min(convolution(x), transformed_geometric_mean(x))
    #     before = integral(convolution_before_same, NEGATIVE_INFINITY, 0)
    #     after = integral(convolution_before_same, 0, POSITIVE_INFINITY)
    #
    #     return before, same, after

# class RelationFormulaConvolution(BaseRelationFormula):
#     def function_convolution_uniform(self, bounds_1, bounds_2):
#         a1, b1 = bounds_1
#         a2, b2 = bounds_2
#         length_1 = fabs(a1 - b1)
#         length_2 = fabs(a2 - b2)
#
#         convolution_bounds_a, convolution_bounds_b = a1 - b2, b1 - a2
#
#         trapezium_0, trapezium_1 = convolution_bounds_a, convolution_bounds_a + min(length_2, length_1)
#         trapezium_2, trapezium_3 = trapezium_1 + fabs(length_1 - length_2), convolution_bounds_b
#         #assert trapezium_2 + min(length_2, length_1) == trapezium_3
#
#         p = min(1 / length_1, 1 / length_2)
#
#         result = FunctionPiecewiseLinear({trapezium_0: 0, trapezium_1: p, trapezium_2: p, trapezium_3: 0},
#                                          FUNCTION_ZERO)
#         result.is_normalised = True
#
#         result.plot().show()
#
#         return result
#
#     def function_convolution(self, dist_1, dist_2, bins=50):
#         a_1, b_1, a_2, b_2 = 0, 0, 0, 0
#         if dist_1 in self.bounds:
#             a_1, b_1 = self.bounds[dist_1]
#         else:
#             a_1, b_1 = calculate_bounds_of_probability_distribution(dist_1)
#             self.bounds[dist_1] = a_1, b_1
#         if dist_2 in self.bounds:
#             a_2, b_2 = self.bounds[dist_2]
#         else:
#             a_2, b_2 = calculate_bounds_of_probability_distribution(dist_2)
#             self.bounds[dist_2] = a_2, b_2
#
#         if (type(dist_1.dist), type(dist_2.dist)) == (uniform_gen, uniform_gen):
#             return self.function_convolution_uniform((a_1, b_1), (a_2, b_2))
#
#         convolution_bounds_a, convolution_bounds_b = min(a_1, a_2), max(b_1, b_2)
#
#         delta = fabs(convolution_bounds_a - convolution_bounds_b) / bins
#         convolution_interval = TimeInterval(convolution_bounds_a, convolution_bounds_b, bins)
#         x = [dist_1.pdf(t) for t in convolution_interval]
#         y = [dist_2.pdf(t) for t in reversed(convolution_interval)]
#
#         c = convolve(x, y)
#         dictionary_convolution = {}
#         for t in xrange(len(c)):
#             dictionary_convolution[delta * t] = c[t]
#         bias = calculateCenterMass(dictionary_convolution)[0] + dist_2.mean() - dist_1.mean()
#         dictionary_convolution_biased = {}
#         for t in dictionary_convolution:
#             dictionary_convolution_biased[t - bias] = dictionary_convolution[t]
#
#         convolution_function = FunctionPiecewiseLinear(dictionary_convolution_biased, FunctionHorizontalLinear(0))
#         return convolution_function.normalised()
#
#     def compare(self, dist_1, dist_2):
#         convolution = self.function_convolution(dist_1, dist_2)
#         a_1, b_1 = self.bounds_of(dist_1)
#         a_2, b_2 = self.bounds_of(dist_2)
#         same_bound = fabs(max(a_1, a_2) - min(b_1, b_2))
#         before = integral(convolution, NEGATIVE_INFINITY, -same_bound)
#         same = integral(convolution, -same_bound, same_bound)
#         after = integral(convolution, same_bound, POSITIVE_INFINITY)
#
#         return before, same, after


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

        if (portion_after + portion_before) == 0:
            pass

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
