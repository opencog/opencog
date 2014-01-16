from math import fabs, sqrt
from numpy import convolve, NINF as NEGATIVE_INFINITY, PINF as POSITIVE_INFINITY
from scipy.stats.distributions import uniform_gen
from spatiotemporal.temporal_events.trapezium_formulas import function_convolution_uniform
from spatiotemporal.temporal_events.util import calculate_bounds_of_probability_distribution
from spatiotemporal.temporal_interval_handling import calculateCenterMass
from spatiotemporal.time_intervals import TimeInterval
from utility.geometric import FunctionComposite, FunctionLinear, FunctionPiecewiseLinear, FunctionHorizontalLinear, integral, FUNCTION_ZERO, FUNCTION_ONE

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
        if dist in self.bounds:
            return self.bounds[dist]

        bounds = calculate_bounds_of_probability_distribution(dist)
        self.bounds[dist] = bounds
        return bounds

    def compare(self, dist_1, dist_2):
        """
        returns before, same and after
        """
        return 0.0, 0.0, 0.0


class FormulaCreator(object):
    def __init__(self, relation_formula):
        if not isinstance(relation_formula, BaseRelationFormula):
            raise TypeError("'relation_formula' should be an instance of BaseTemporalFormula")
        self.relation_formula = relation_formula

    def temporal_relations_between(self, temporal_event_1, temporal_event_2):
        beginning_a, ending_a = temporal_event_1.distribution_beginning, temporal_event_1.distribution_ending
        beginning_b, ending_b = temporal_event_2.distribution_beginning, temporal_event_2.distribution_ending
        self.relation_formula.bounds[beginning_a] = temporal_event_1.a, temporal_event_1.beginning
        self.relation_formula.bounds[ending_a] = temporal_event_1.ending, temporal_event_1.b
        self.relation_formula.bounds[beginning_b] = temporal_event_2.a, temporal_event_2.beginning
        self.relation_formula.bounds[ending_b] = temporal_event_2.ending, temporal_event_2.b

        before = {}
        same = {}
        after = {}

        combinations = [
            (beginning_a, beginning_b),
            (beginning_a, ending_b),
            (ending_a, beginning_b),
            (ending_a, ending_b)
        ]

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

        return result


class RelationFormulaConvolution(BaseRelationFormula):
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
            return function_convolution_uniform((a_1, b_1), (a_2, b_2))

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

    def compare(self, dist_1, dist_2):
        convolution = self.function_convolution(dist_1, dist_2)
        a_1, b_1 = self.bounds[dist_1]
        a_2, b_2 = self.bounds[dist_2]
        same_bound = fabs(max(a_1, a_2) - min(b_1, b_2))
        before = integral(convolution, NEGATIVE_INFINITY, -same_bound)
        same = integral(convolution, -same_bound, same_bound)
        after = integral(convolution, same_bound, POSITIVE_INFINITY)

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
        if distance == 0:
            if delta < 0:
                portion_after, portion_before = 0.0, 1.0
        else:
            dist_1_standardized_pdf = lambda x: dist_1.pdf(dist_1_standard_deviation * x + dist_1_mean)
            dist_2_standardized_pdf = lambda x: dist_2.pdf(dist_2_standard_deviation * x + dist_2_mean)
            geometric_mean = lambda t: sqrt(dist_1_standardized_pdf(t) * dist_2_standardized_pdf(t))
            geometric_mean_scaled = lambda p: geometric_mean(p / distance)
            geometric_mean_scaled_length = min(self.duration_of(dist_1), self.duration_of(dist_2))

            dictionary_input_output = {}
            for time_step in TimeInterval(-geometric_mean_scaled_length / 2.0, geometric_mean_scaled_length / 2.0):
                dictionary_input_output[time_step] = geometric_mean_scaled(time_step)

            geometric_mean_scaled = FunctionPiecewiseLinear(dictionary_input_output, function_undefined=FUNCTION_ZERO)
            portion_after = integral(geometric_mean_scaled, NEGATIVE_INFINITY, delta)
            portion_before = integral(geometric_mean_scaled, delta, POSITIVE_INFINITY)

        after = portion_after / (portion_after + portion_before) * non_same_portion
        return 1.0 - same - after, same, after


class RelationFormulaTrapezium(BaseRelationFormula):
    def compare(self, dist_1, dist_2):
        a_1, b_1 = self.bounds_of(dist_1)
        a_2, b_2 = self.bounds_of(dist_2)

        dist_1_duration, dist_2_duration = self.duration_of(dist_1), self.duration_of(dist_2)
        dist_1_uniform_probability = 1.0 / dist_1_duration
        dist_2_uniform_probability = 1.0 / dist_2_duration

        same_a = max(a_1, a_2)
        same_b = min(b_1, b_2)

        same = 0
        if same_a < same_b:
            same = sqrt(dist_1_uniform_probability * dist_2_uniform_probability) * (same_b - same_a)

        l = fabs(dist_1_duration - dist_2_duration)

        dictionary_bounds_function = {(l / 2.0, POSITIVE_INFINITY): FUNCTION_ONE}
        if l > 0:
            dictionary_bounds_function[(-l / 2.0, l / 2.0)] = FunctionLinear(x_0=- l / 2.0, y_0=0, x_1=l / 2.0, y_1=1)
        proportion_function = FunctionComposite(dictionary_bounds_function, function_undefined=FUNCTION_ZERO)

        non_same_portion = 1.0 - same
        after = proportion_function(dist_1.mean() - dist_2.mean()) * non_same_portion
        return 1 - same - after, same, after

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from scipy.stats import norm, uniform, expon
    from spatiotemporal.temporal_events import TemporalEvent, TemporalEventPiecewiseLinear
    import matplotlib.pyplot as plt

    a = uniform(loc=1000, scale=2)
    m, s, k = a.stats(moments='msk')
    print m, s, k
    quit()

    figure_number = 1
    for event_1, event_2 in [
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
        #     TemporalEvent(uniform(loc=4, scale=1), uniform(loc=11, scale=2)),
        #     TemporalEvent(uniform(loc=0, scale=7), uniform(loc=8, scale=7))
        # ),

        (
            TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4)),
            TemporalEvent(uniform(loc=0, scale=11), uniform(loc=13, scale=4))
        ),

        # (
        #     TemporalEvent(uniform(loc=1, scale=8), uniform(loc=6, scale=8)),
        #     TemporalEvent(uniform(loc=0, scale=22), uniform(loc=13, scale=8))
        # ),

        # (
        #     TemporalEvent(uniform(loc=2, scale=2), uniform(loc=7, scale=2)),
        #     TemporalEvent(uniform(loc=1, scale=4), uniform(loc=6, scale=4))
        # ),
        #
        # (
        #    TemporalEvent(uniform(loc=1, scale=2), uniform(loc=4, scale=2)),
        #    TemporalEvent(uniform(loc=6, scale=2), uniform(loc=9, scale=2))
        # ),
        #
        # (
        #    TemporalEvent(uniform(loc=0, scale=3), uniform(loc=15, scale=2)),
        #    TemporalEvent(uniform(loc=5, scale=2), uniform(loc=9, scale=3))
        # ),
        #
        # (
        #    TemporalEvent(uniform(loc=5, scale=3), uniform(loc=9, scale=2)),
        #    TemporalEvent(uniform(loc=1, scale=2), uniform(loc=15, scale=3))
        # ),
        #
        # (
        #    TemporalEvent(uniform(loc=0, scale=2), uniform(loc=10, scale=2)),
        #    TemporalEvent(uniform(loc=15, scale=2), uniform(loc=25, scale=2))
        # ),
        #
        # (
        #    TemporalEvent(uniform(loc=15, scale=2), uniform(loc=25, scale=2)),
        #    TemporalEvent(uniform(loc=0, scale=2), uniform(loc=10, scale=2))
        # ),
        #
        # (
        #    TemporalEvent(norm(loc=1, scale=4.5), expon(loc=30, scale=2)),
        #    TemporalEvent(norm(loc=25, scale=4.5), expon(loc=60, scale=2))
        # ),
        #
        # (
        #    TemporalEvent(expon(loc=1, scale=4.5), norm(loc=30, scale=2)),
        #    TemporalEvent(expon(loc=25, scale=4.5), norm(loc=60, scale=2))
        # ),
        #
        # (
        #    TemporalEventPiecewiseLinear({1: 0, 2: 0.1, 3: 0.3, 4: 0.7, 5: 1}, {6: 1, 7: 0.9, 8: 0.6, 9: 0.1, 10: 0}),
        #    TemporalEventPiecewiseLinear({7.5: 0, 8.5: 0.1, 9.5: 0.3, 10.5: 0.7, 11.5: 1},
        #                                 {13: 1, 14.5: 0.9, 15.3: 0.6, 17: 0.1, 20: 0})
        # ),
    ]:
        tfc = TemporalFormulaConvolution()
        fc = FormulaCreator(tfc)

        temporal_relations = event_1 * event_2

        print '\nFigure' + str(figure_number)
        print '----------------------'
        print sum(temporal_relations.values())
        for p in 'pmoFDseSdfOMP':
            print p, temporal_relations[p]

        figure_number += 1

        event_1.plot().ylim(ymin=-0.1, ymax=1.1)
        event_2.plot().figure()

    plt.show()
