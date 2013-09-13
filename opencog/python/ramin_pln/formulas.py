__author__ = 'ramin'

from opencog.atomspace import TruthValue

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0

REVISION_STRENGTH_DEPENDENCY = 0.0
REVISION_COUNT_DEPENDENCY = 0.0


def denominator(value):
    """

    :param value:
    :return:
    """
    return max(value, 0.00001)


def deduction(tvA, tvB, tvC, tvAB, tvBC):
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


def inversion(tvA, tvB, tvAB):
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


def revision(tvA, tvB):
    """

    :param tvA:
    :param tvB:
    :return:
    """
    (sA, nA) = (tvA.mean, tvA.count)
    (sB, nB) = (tvB.mean, tvB.count)

    wA = nA / denominator(nA + nB)
    wB = nB / denominator(nA + nB)

    c_strength = REVISION_STRENGTH_DEPENDENCY
    c_count = REVISION_COUNT_DEPENDENCY

    s3 = (wA * sA + wB * sB - c_strength * sA * sB)
    n3 = max(nA, nB) + c_count * min(nA, nB)

    return TruthValue(s3, n3)