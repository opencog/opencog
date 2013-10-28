from opencog.atomspace import count_to_confidence, confidence_to_count, TruthValue

import operator, functools, itertools

from utility.util import concat_lists

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 1.0
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 1.0
MembershipToExtensionalInheritanceCountDiscountFactor = 1.0
CRISP_COUNT_THRESHOLD = 0.97

def identityFormula(tvs):
    return tvs

def tv_seq_to_tv_tuple_seq(tvs):
    return [(tv.mean, tv.count) for tv in tvs]

def deductionSimpleFormula(tvs):
    [(sAB, nAB), (sBC, nBC), (_, nA), (sB, nB),  (sC, _)] = tv_seq_to_tv_tuple_seq(tvs)

    # Temporary filtering fix to make sure that nAB >= nA
    nA = min(nA, nAB)
    sDenominator = low(1 - sB)
    nDenominator = low(nB)
    
    w1 = DEDUCTION_TERM_WEIGHT # strength
    w2 = 2 - w1 # strength
    sAC = (w1 * sAB * sBC
               + w2 * (1 - sAB) * (sC - sB * sBC) / sDenominator)
    
    nAC = INDEPENDENCE_ASSUMPTION_DISCOUNT * nA * nBC / nDenominator
    
    return [TruthValue(sAC, nAC)]

# better deduction formula based on concept geometry
def deductionGeometryFormula(tvs):
    [AB, BC] = tvs

    sAC = AB.mean*BC.mean / min(AB.mean+BC.mean, 1)
    nAC = AB.count+BC.count

    return [TruthValue(sAC, nAC)]

def inversionFormula(tvs):
    [(sAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)
    
    sBA = sAB * sA / low(sB)
    nBA = nAB * nB / low(nA)
    
    return [TruthValue(sBA, nBA)]

def crispModusPonensFormula(tvs):
    (sAB, nAB), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    true = 0.5
    if all(x > true for x in [sAB, nAB, sA, nA]):
        return [TruthValue(1, confidence_to_count(0.99))]
    else:
        return [TruthValue(0, 0)]

def modusPonensFormula(tvs):
    (sAB, nAB), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    if nAB > CRISP_COUNT_THRESHOLD and nA > CRISP_COUNT_THRESHOLD:
        return crispModusPonensFormula(tvs)

    # P(B|not A) -- how should we find this?
    #BNA = TruthValue(0.5, 0.01)
    sBNA, nBNA = (0.5, 0.01)
    
    n2 = min(nAB, nA)
    if n2 + nBNA > 0:
        s2 = ((sAB * sA * n2 + nBNA +
                 sBNA * (1 - sA) * nBNA) /
                 low(n2 + nBNA))
    else:
        raise NotImplementedError
        s2 = BNA.confidence
    
    return [TruthValue(s2, n2)]

def inheritanceFormula(tvs):
    tv_subset, tv_inh = tvs

    # simple average of subset and inheritance
    mean = (tv_subset.mean + tv_inh.mean) /2.0
    count = (tv_subset.count + tv_inh.count) / 2.0

    return [TruthValue(mean, count)]

def notFormula(tvs):
    tv = tvs[0]
    return [TruthValue(1.0 - tv.mean, tv.count)]

def andSymmetricFormula(tvs):
    total_strength = 1.0
    total_confidence = 1.0
    
    for tv in tvs:
        total_strength *= tv.mean
        total_confidence *= count_to_confidence(tv.count)
    
    return [TruthValue(total_strength, confidence_to_count(total_confidence))]

def orFormula(tvs):
    N = len(tvs)
    
    if N == 1:
        return [tvs[0]]
    
    if N > 2:
        # TODO handle via divide-and-conquer or something
        raise NotImplementedError("OR calculation not supported for arity > 2")

    (sA, nA), (sB, nB) = tv_seq_to_tv_tuple_seq(tvs)
    
    A = sA * nB
    B = sB * nA
    
    s_tot = sA + sB
    n_tot = nA + nB - (A + B) / 2
    
    return [TruthValue(s_tot, n_tot)]

def andPartitionFormula(tvs, U):
    [(sAndA, nAndA), (sAndB, nAndB)] = tvs

    s = sAndA * sAndB
    n = nAndA + nAndB
    return [TruthValue(s, n)]

def ext2InhFormula(tvs, U):
    [(sAB, nAB)] = tvs
    
    sABint = sAB
    nABint = nAB * EXTENSION_TO_INTENSION_DISCOUNT_FACTOR
    return [TruthValue(sABint, nABint)]

def inheritanceFormula(tvs, U):
    [(sExt, nExt), (sInt, nInt)] = tvs

    s = (sExt + sInt) / 2.0
    n = (nExt + nInt) / 2.0

    return [TruthValue(s, n)]

def inheritance2SimilarityFormula(tvs, U):
    [(sAB, nAB), (sBA, nBA)] = tvs

    s = 1.0/ ( 1.0/sAB + 1.0/sBA -1)
    n = (nAB + nBA) / (1 + s)

    return [TruthValue(s, n)]

def mem2InhFormula(tvs):
    [mem_tv] = tvs
    count = mem_tv.count * MembershipToExtensionalInheritanceCountDiscountFactor

    return [TruthValue(mem_tv.mean, count)]

def subsetEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean > 0.5
    mem_b = mem_b_tv.mean > 0.5

    # P(x in B | x in A)

    if not mem_a:
        # Irrelevant
        return [TruthValue(0, 0)]
    elif mem_b:
        # A and B => 1 observation of B|A
        return [TruthValue(1, 1)]
    else:
        # A and NOTB => 1 observation of NOTB|A
        return [TruthValue(0, 1)]

def similarityEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean > 0.5
    mem_b = mem_b_tv.mean > 0.5

    # tv = |A and B| / |A or B|

    if not (mem_a or mem_b):
        # not an observation of (A or B)
        return [TruthValue(0, 0)]
    elif mem_a and mem_b:
        # increment |A and B| as well as |A or B|
        return [TruthValue(1, 1)]
    else:
        # increment |A or B| without changing |A and B|
        return [TruthValue(0, 1)]

def extensionalEvaluationFormula(tvs):
    '''Inputs: Membership x A.tv, Membership x B.tv
Outputs: SubsetLink A B.tv, SubsetLink B A.tv, SimilarityLink A B.tv'''
    subsetAB = subsetEvaluationFormula(tvs)
    subsetBA = subsetEvaluationFormula(reversed(tvs))

    similarityAB = similarityEvaluationFormula(tvs)

    # Each of those formulas returns a list containing one TV, and this formula returns a list containing 3 TVs
    tvs = subsetAB + subsetBA + similarityAB
    for tv in tvs: print str(tv)
    return tvs

def extensionalSimilarityFormula(tvs):
    [and_tv, or_tv] = tvs

    # calculate the size of the two sets A AND B vs A OR B
    and_size = 1.0*and_tv.mean*and_tv.count
    or_size  = 1.0*or_tv.mean*or_tv.count

    if or_size == 0:
        return [TruthValue(0, 0)]

    P = and_size / or_size
    N = and_tv.count + or_tv.count

    return [TruthValue(P, N)]

def attractionFormula(tvs):
    [ab, b] = tvs

    mean = min(0, ab.mean - b.mean)

    count = ab.count

    return [TruthValue(mean, count)]

def revisionFormula(tvs):
    x, y = tvs
    # revise two truth values

    n = x.count+y.count
    weight_1 = x.count*1.0/n
    weight_2 = y.count*1.0/n
    # TODO maybe check for overlap
    s = (weight_1*x.mean + weight_2*y.mean)
    return TruthValue(s, n)

def andBreakdownFormula(tvs):
    [A, AND_AB] = tvs

    sB = AND_AB.mean / A.mean
    nB = 1 # bizarbitrary count to symbolize how innacurate this rule is!

    return [TruthValue(sB, nB)]

def orBreakdownFormula(tvs):
    [A, OR_AB] = tvs

    sB = OR_AB.mean / (1-A.mean)
    nB = 1 # bizarbitrary count to symbolize how innacurate this rule is!

    return [TruthValue(sB, nB)]

def low(n):
    return max(n, 0.00001)


