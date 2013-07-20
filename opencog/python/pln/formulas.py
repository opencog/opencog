__author__ = 'ramin'

from opencog.atomspace import TruthValue

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0


def denominator(value):
    return max(value, 0.00001)


def deduce(tvA, tvB, tvC, tvAB, tvBC):
    """

    :param tvA:
    :param tvB:
    :param tvC:
    :param tvAB:
    :param tvBC:
    :return:
    """
    (sA, nA) = (tvA.mean, tvA.count)
    (sB, nB) = (tvB.mean, tvB.count)
    (sC, nC) = (tvC.mean, tvC.count)
    (sAB, nAB) = (tvAB.mean, tvAB.count)
    (sBC, nBC) = (tvBC.mean, tvBC.count)

    w1 = DEDUCTION_TERM_WEIGHT
    w2 = 2 - w1

    sAC = w1*sAB*sBC + (w2*(1-sAB)*(sC-sB*sBC) / denominator(1-sB))
    nAC = INDEPENDENCE_ASSUMPTION_DISCOUNT*nA*nBC / denominator(nB)

    return TruthValue(sAC, nAC)


def inverse(tvA, tvB, tvAB):
    """

    :param tvA:
    :param tvB:
    :param tvAB:
    :return:
    """
    (sA, nA) = (tvA.mean, tvA.count)
    (sB, nB) = (tvB.mean, tvB.count)
    (sAB, nAB) = (tvAB.mean, tvAB.count)

    sBA = sAB * sA / denominator(sB)
    nBA = nAB * nB / denominator(nA)

    return TruthValue(sBA, nBA)
