from math import sqrt
from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaGeometricMean
from utility.functions import almost_equals

__author__ = 'keyvan'

epsilon = 0.05
uniform_reference = uniform(0, 1)


class BaseCombinationFormula(object):
    def combine(self, relation_a_b, relation_a_d, relation_b_c, relation_d_c):
        pass


class CombinationFormulaGeometricMeanTrapezium(BaseCombinationFormula):
    relation_formula = RelationFormulaGeometricMean()

    def __init__(self):
        pass

    def combine(self, relation_a_b_beginning, relation_a_b_ending, relation_b_beginning_c, relation_b_ending_c):
        before_a_b, same_a_b, after_a_b = relation_a_b_beginning
        before_b_c, same_b_c, after_b_c = relation_b_beginning_c
        unknown = 1.0 / 3

        if almost_equals(before_a_b + before_b_c, 2.0, epsilon)\
                or almost_equals(before_a_b + same_b_c, 2.0, epsilon)\
                or almost_equals(same_a_b + before_b_c, 2.0, epsilon):
            return [(1.0, 0.0, 0.0)]

        if almost_equals(after_a_b + after_b_c, 2.0, epsilon)\
                or almost_equals(after_a_b + same_b_c, 2.0, epsilon)\
                or almost_equals(same_a_b + after_b_c, 2.0, epsilon):
            return [(1.0, 0.0, 0.0)]

        if almost_equals(same_a_b + same_b_c, 2.0, epsilon):
            return [(0.0, 1.0, 0.0)]

        if almost_equals(before_a_b + after_b_c, 2.0, epsilon) or almost_equals(after_a_b + before_b_c, 2.0, epsilon):
            return [(unknown, unknown, unknown)]

        relation_a_c_possibilities = []

        a_possibilities, b_ending_1 = self.unpack(relation_a_b_beginning, relation_a_b_ending)
        c_possibilities, b_ending_2 = self.unpack(tuple(reversed(relation_b_beginning_c)), tuple(reversed(relation_b_ending_c)))

        if len(a_possibilities) == 0:
            if before_a_b == 0:
                if len(c_possibilities) == 1:
                    if before_b_c == 0:
                        relation_a_c_possibilities.append((0.0, 0.0, 1.0))
                    else:
                        relation_a_c_possibilities.append((unknown, unknown, unknown))
                else:
                    relation_a_c_possibilities.append((unknown, unknown, unknown))
                    relation_a_c_possibilities.append((0.0, 0.0, 1.0))
            else:
                if len(c_possibilities) == 1:
                    if after_b_c == 0:
                        relation_a_c_possibilities.append((1.0, 0.0, 0.0))
                    else:
                        relation_a_c_possibilities.append((unknown, unknown, unknown))
                else:
                    relation_a_c_possibilities.append((unknown, unknown, unknown))
                    relation_a_c_possibilities.append((1.0, 0.0, 0.0))

        if len(c_possibilities) == 0:
            if before_b_c == 0:
                if len(a_possibilities) == 1:
                    if before_a_b == 0:
                        relation_a_c_possibilities.append((0.0, 0.0, 1.0))
                    else:
                        relation_a_c_possibilities.append((unknown, unknown, unknown))
                else:
                    relation_a_c_possibilities.append((unknown, unknown, unknown))
                    relation_a_c_possibilities.append((0.0, 0.0, 1.0))
            else:
                if len(a_possibilities) == 1:
                    if after_a_b == 0:
                        relation_a_c_possibilities.append((1.0, 0.0, 0.0))
                    else:
                        relation_a_c_possibilities.append((unknown, unknown, unknown))
                else:
                    relation_a_c_possibilities.append((unknown, unknown, unknown))
                    relation_a_c_possibilities.append((1.0, 0.0, 0.0))

        for possible_a in a_possibilities:
            for possible_c in c_possibilities:
                relation_a_c_possibilities.append(self.relation_formula.compare(possible_a, possible_c))

        return relation_a_c_possibilities

    def _find_c_in_the_middle(self, relation_b_beginning_c, relation_b_ending_c, b_ending):
        before_b_beginning_c, same_b_beginning_c, after_b_beginning_c = relation_b_beginning_c
        before_b_ending_c, same_b_ending_c, after_b_ending_c = relation_b_ending_c
        b_ending_start_point, b_ending_end_point = b_ending.args[0], b_ending.args[0] + b_ending.args[1]

        c_start_point = -(same_b_beginning_c *
                          (before_b_ending_c * (b_ending_start_point + b_ending_end_point)
                           + b_ending_start_point + before_b_ending_c)) / (before_b_ending_c *
                                                                           after_b_beginning_c + same_b_beginning_c
                                                                           * (1 + before_b_ending_c))

        c_end_poin = 1 + after_b_beginning_c * c_start_point / same_b_beginning_c

        return uniform(c_start_point, c_end_poin - c_start_point)

    def unpack_partial(self, relation, length_b=1.0):
        before, same, after = relation
        if same == 0:
            return []

        length_a = 1.0 / same ** 2

        if before > 0 and after > 0:
            possibilities = []
            before_portion = before / (before + after)
            possibilities.append(uniform((1 - length_a) * before_portion, length_a))
            length_a = same ** 2
            possibilities.append(uniform((1 - length_a) * (1 - before_portion), length_a))
            return possibilities

        length_a = (same ** 4 + 1) / (2 * same ** 2)    # average case

        if before > 0:
            return [uniform(same * sqrt(length_a) - length_a, length_a)]

        if after > 0:
            return [uniform(1 - same * sqrt(length_a), length_a)]

        return [uniform(0, 1)]

    def unpack(self, relation_a_b_beginning, relation_a_b_ending):
        before_a_b_beginning, same_a_b_beginning, after_a_b_beginning = relation_a_b_beginning
        before_a_b_ending, same_a_b_ending, after_a_b_ending = relation_a_b_ending

        if almost_equals(before_a_b_beginning + same_a_b_beginning + same_a_b_ending + after_a_b_ending, 2.0, epsilon):
            return [], uniform_reference   # Inconsistent

        if almost_equals(before_a_b_beginning + before_a_b_ending, 2.0, epsilon):
            return [], uniform_reference

        if almost_equals(after_a_b_beginning + after_a_b_ending, 2.0, epsilon):
            return [], uniform_reference

        if almost_equals(before_a_b_ending, 1.0, epsilon):
            return self.unpack_partial(relation_a_b_beginning), uniform_reference

        if almost_equals(after_a_b_beginning, 1.0, epsilon):
            return self.unpack_partial(relation_a_b_ending), uniform_reference

        a_possibilities = self.unpack_partial(relation_a_b_beginning)
        if len(a_possibilities) == 1:
            a_possibility = a_possibilities[0]

            return
        else:
            a_possibility_1, a_possibility_2 = a_possibilities
            a_possibility = a_possibility_1
            if a_possibility.args[1] < a_possibility_2.args[1]:
                a_possibility = a_possibility_2

        a_start_point, length_a = a_possibility.args

        if before_a_b_ending * same_a_b_ending * after_a_b_ending > 0:
            if almost_equals(before_a_b_beginning, 0, epsilon):
                length_b_ending = length_a * same_a_b_ending ** 2
            else:
                length_b_ending = same_a_b_ending ** 2 / same_a_b_beginning ** 2
            b_ending = uniform(a_start_point + before_a_b_ending * (length_a - length_b_ending) /
                               (before_a_b_ending + after_a_b_ending), length_b_ending)
            return [a_possibility], b_ending
        else:
            denominator = length_a * same_a_b_ending ** 2
            length_b_ending_lower_bound = (length_a * same_a_b_ending ** 2) ** 2 / denominator
            length_b_ending_upper_bound = (a_start_point + length_a - 1) ** 2 / denominator
            length_b_ending = (length_b_ending_lower_bound + length_b_ending_upper_bound) / 2.0
            b_start_point = a_start_point + length_a - same_a_b_ending * sqrt(length_b_ending * length_a)
            b_ending = uniform(b_start_point, length_b_ending)
            return [a_possibility], b_ending



if __name__ == '__main__':
    b_ending = uniform(3, 4)
    c = uniform(0.5, 3)
    rf = RelationFormulaGeometricMean()
    f = CombinationFormulaGeometricMeanTrapezium()
    r_1, r_2 = rf.compare(c, uniform_reference), rf.compare(c, b_ending)
    c = f._find_c_in_the_middle(r_1, r_2, b_ending)
    print c.args
    quit()




    from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium
    # f = CombinationFormulaGeometricMeanTrapezium()
    # rf = RelationFormulaGeometricMean()
    # a, b_beg, b_end = uniform(5, 2), uniform(1, 3), uniform(8, 4)
    # [a_h], b_end_h = f.unpack(rf.compare(a, b_beg), rf.compare(a, b_end))
    #
    # print a_h.args, b_end_h.args
    # print rf.compare(a, b_beg), rf.compare(a, b_end)
    # print rf.compare(a_h, uniform_reference), rf.compare(a_h, b_end_h)
    #
    # quit()

    A = TemporalEventTrapezium(2, 7, 5, 6)
    B = TemporalEventTrapezium(1, 6, 3, 4)
    C = TemporalEventTrapezium(0, 9, 7, 8)

    f = CombinationFormulaGeometricMeanTrapezium()
    rf = RelationFormulaGeometricMean()
    # print rf.compare(f.unpack((1, 0, 0))[0], uniform(0, 1))

    A_, B_beg, B_end, C_ = A.distribution_beginning, B.distribution_beginning, B.distribution_ending, C.distribution_beginning
    # A, B, C = C.distribution_beginning, B.distribution_beginning, A.distribution_beginning
    print f.combine(rf.compare(A_, B_beg),
                    rf.compare(A_, B_end),
                    rf.compare(B_beg, C_),
                    rf.compare(B_end, C_))
    print rf.compare(A_, C_)

    plt = A.plot()
    B.plot(plt)
    C.plot(plt)
    plt.show()
