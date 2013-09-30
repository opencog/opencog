from opencog.atomspace import count_to_confidence, confidence_to_count, TruthValue

import operator, functools, itertools

from utility.util import concat_lists

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 1.0
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 1.0

def identityFormula(tvs):
    [(sA, nA)] = tvs
    
    return (sA, nA)

# Won't work with the current control
#def trueFormula(tvs, U):
#    [] = tvs
#    
#    return (1.0, confidence_to_count(1.0))

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

def inversionFormula(tvs):
    [(sAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)
    
    sBA = sAB * sA / low(sB)
    nBA = nAB * nB / low(nA)
    
    return [TruthValue(sBA, nBA)]

def crispModusPonensFormula(tvs, U):
    (sAB, nAB), (sA, nA) = tvs

    true = 0.1
    if all(x > true for x in [sAB, nAB, sA, nA]):
        return (1, confidence_to_count(1))
    else:
        return (0, 0)

def modusPonensFormula(tvs, U):
    (sAB, nAB), (sA, nA) = tvs

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
    
    return (s2, n2)

def notFormula(tvs):
    tv = tvs[0]
    return (1.0 - tv.mean, tv.count)

def andSymmetricFormula(tvs):
    total_strength = 1.0
    total_confidence = 1.0
    
    for tv in tvs:
        total_strength *= tv.mean
        total_confidence *= count_to_confidence(tv.count)
    
    return (total_strength, confidence_to_count(total_confidence))

def orFormula(tvs):
    N = len(tvs)
    
    if N == 1:
        return tvs[0]
    
    if N > 2:
        # TODO handle via divide-and-conquer or something
        raise NotImplementedError("OR calculation not supported for arity > 2")

    (sA, nA), (sB, nB) = tvs
    
    A = sA * nB
    B = sB * nA
    
    s_tot = sA + sB
    n_tot = nA + nB - (A + B) / 2
    
    return (s_tot, n_tot)

def andPartitionFormula(tvs, U):
    [(sAndA, nAndA), (sAndB, nAndB)] = tvs

    s = sAndA * sAndB
    n = nAndA + nAndB
    return (s, n)

def ext2InhFormula(tvs, U):
    [(sAB, nAB)] = tvs
    
    sABint = sAB
    nABint = nAB * EXTENSION_TO_INTENSION_DISCOUNT_FACTOR
    return (sABint, nABint)

def inheritanceFormula(tvs, U):
    [(sExt, nExt), (sInt, nInt)] = tvs

    s = (sExt + sInt) / 2.0
    n = (nExt + nInt) / 2.0

    return (s, n)

def inheritance2SimilarityFormula(tvs, U):
    [(sAB, nAB), (sBA, nBA)] = tvs

    s = 1.0/ ( 1.0/sAB + 1.0/sBA -1)
    n = (nAB + nBA) / (1 + s)

    return (s, n)

def mem2InhFormula(tvs):
    [mem_tv] = tvs
    count = mem_tv.count * MembershipToExtensionalInheritanceCountDiscountFactor

    return TruthValue(mem_tv.mean, count)

def revisionFormula(tvs):
    x, y = tvs
    # revise two truth values

    n = x.count+y.count
    ## it should be a confidence-weighted average
    #weight_1 = x.count*1.0/n
    #weight_2 = y.count*1.0/n
    # TODO maybe check for overlap
#    s = (weight_1*x.mean+y.mean)/2.0
    s = (x.mean+y.mean)/2.0
    return TruthValue(s, n)

def low(n):
    return max(n, 0.00001)

# temporal formulas
def beforeFormula(dist1, dist2):
    times_event1 = [int(t) for t in dist1.keys()]
    times_event2 = [int(t) for t in dist2.keys()]
    
    if all(t_event1 < t_event2 for t_event1 in times_event1 for t_event2 in times_event2):
        strength = 1
    else:
        strength = 0
    
    return strength
