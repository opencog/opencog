from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaConvolution
import numpy
from lmfit import minimize, Parameters
from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium, generate_random_events

__author__ = 'keyvan'

formula = RelationFormulaConvolution()


class CompositionFitter(object):
    def __init__(self, data_a_b, data_b_c):
        self.data = numpy.array(data_a_b + data_b_c)
        self.parameters = Parameters()
        self.parameters.add_many(
            ('A_beg_loc', 0, True, None, None, None),
            ('A_beg_scale', 11, False, 0.0, None, None),
            # ('delta_A', -1, True, None, 0.0, None),
            # ('A_end_loc', None, False, None, None, '(A_beg_loc + A_beg_scale) - delta_A'),
            ('A_end_loc', 12, True, None, None, None),
            ('A_end_scale', 12, False, 0.0, None, None),
            ('B_beg_loc', 10, False, None, None, None),
            ('B_beg_scale', 10, False, 0.0, None, None),
            # ('delta_B', -1, True, None, 0.0, None),
            # ('B_end_loc', None, False, None, None, '(B_beg_loc + B_beg_scale) - delta_B'),
            ('B_end_loc', 21, True, None, None, None),
            ('B_end_scale', 5, False, 0.0, None, None),
            ('C_beg_loc', 15, True, None, None, None),
            ('C_beg_scale', 2, False, 0.0, None, None),
            # ('delta_C', -1, True, None, 0.0, None),
            # ('C_end_loc', None, False, None, None, '(C_beg_loc + C_beg_scale) - delta_C'),
            ('C_end_loc', 15, True, None, None, None),
            ('C_end_scale', 10, True, 0.0, None, None)
        )

        result = minimize(self.fitness, self.parameters)
        print result.residual

    def fitness(self, parameters):
        model = numpy.zeros(16)
        A_beg_loc = parameters['A_beg_loc'].value
        A_beg_scale = parameters['A_beg_scale'].value
        A_end_loc = parameters['A_end_loc'].value
        A_end_scale = parameters['A_end_scale'].value
        B_beg_loc = parameters['B_beg_loc'].value
        B_beg_scale = parameters['B_beg_scale'].value
        B_end_loc = parameters['B_end_loc'].value
        B_end_scale = parameters['B_end_scale'].value
        C_beg_loc = parameters['C_beg_loc'].value
        C_beg_scale = parameters['C_beg_scale'].value
        C_end_loc = parameters['C_end_loc'].value
        C_end_scale = parameters['C_end_scale'].value

        A_beg = uniform(A_beg_loc, A_beg_scale)
        A_end = uniform(A_end_loc, A_end_scale)
        B_beg = uniform(B_beg_loc, B_beg_scale)
        B_end = uniform(B_end_loc, B_end_scale)
        C_beg = uniform(C_beg_loc, C_beg_scale)
        C_end = uniform(C_end_loc, C_end_scale)

        i = 0
        for dist_1_current, dist_2_current in [
            (dist_1, dist_2)
            for dist_1 in [A_beg, A_end]
            for dist_2 in [B_beg, B_end]
        ] + [
            (dist_1, dist_2)
            for dist_1 in [B_beg, B_end]
            for dist_2 in [C_beg, C_end]
        ]:
            before, same, after = formula.compare(dist_1_current, dist_2_current)
            model[i] = before
            model[i + 1] = same
            i += 2

        return model - self.data

    def get_trapeziums(self):
        A_beg_loc = self.parameters['A_beg_loc'].value
        A_beg_scale = self.parameters['A_beg_scale'].value
        A_end_loc = self.parameters['A_end_loc'].value
        A_end_scale = self.parameters['A_end_scale'].value
        B_beg_loc = self.parameters['B_beg_loc'].value
        B_beg_scale = self.parameters['B_beg_scale'].value
        B_end_loc = self.parameters['B_end_loc'].value
        B_end_scale = self.parameters['B_end_scale'].value
        C_beg_loc = self.parameters['C_beg_loc'].value
        C_beg_scale = self.parameters['C_beg_scale'].value
        C_end_loc = self.parameters['C_end_loc'].value
        C_end_scale = self.parameters['C_end_scale'].value
        A = TemporalEventTrapezium(A_beg_loc, A_end_loc + A_end_scale, A_beg_loc + A_beg_scale, A_end_loc)
        B = TemporalEventTrapezium(B_beg_loc, B_end_loc + B_end_scale, B_beg_loc + B_beg_scale, B_end_loc)
        C = TemporalEventTrapezium(C_beg_loc, C_end_loc + C_end_scale, C_beg_loc + C_beg_scale, C_end_loc)

        return A, B, C

if __name__ == '__main__':
    from spatiotemporal.temporal_events.composition.non_linear_least_squares import DecompositionFitter
    import time

    # a = TemporalEventTrapezium(0, 4, 1.5, 3)
    # b = TemporalEventTrapezium(0, 8, 1, 3.5)
    # c = TemporalEventTrapezium(0.2, 3.8, 0.9, 3.1)
    a = TemporalEventTrapezium(1, 28, 12, 16)
    b = TemporalEventTrapezium(10, 26, 20, 21)
    c = TemporalEventTrapezium(22, 30, 24, 25)

    print (a * b).to_list()
    print (b * c).to_list()
    print (a * c).to_list()
    start = time.time()
    fitter = CompositionFitter(DecompositionFitter(a*b).get_composition_data(),
                               DecompositionFitter(b*c).get_composition_data())
    print 'time:', time.time() - start
    a, b, c = fitter.get_trapeziums()
    print
    print (a * b).to_list()
    print (b * c).to_list()
    print (a * c).to_list()

    print float(a.a), float(a.b), float(a.beginning), float(a.ending)
    print float(b.a), float(b.b), float(b.beginning), float(b.ending)
    print float(c.a), float(c.b), float(c.beginning), float(c.ending)
