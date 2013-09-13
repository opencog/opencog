try:
    from opencog.atomspace import count_to_confidence, confidence_to_count
except ImportError:
    from atomspace_remote import count_to_confidence, confidence_to_count

import operator, functools, itertools

from utility.util import concat_lists

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 1.0
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 1.0

def identityFormula(tvs, U):
    [(sA, nA)] = tvs
    
    return (sA, nA)

# Won't work with the current control
#def trueFormula(tvs, U):
#    [] = tvs
#    
#    return (1.0, confidence_to_count(1.0))

def deductionSimpleFormula(tvs, U):
    (sAB, nAB), (sBC, nBC), (_, nA), (sB, nB),  (sC, _) = tvs

    # Temporary filtering fix to make sure that nAB >= nA
    nA = min(nA, nAB)
    sDenominator = low(1 - sB)
    nDenominator = low(nB)
    
    w1 = DEDUCTION_TERM_WEIGHT # strength
    w2 = 2 - w1 # strength
    sAC = (w1 * sAB * sBC
               + w2 * (1 - sAB) * (sC - sB * sBC) / sDenominator)
    
    nAC = INDEPENDENCE_ASSUMPTION_DISCOUNT * nA * nBC / nDenominator
    
    return (sAC, nAC)

def inversionFormula(tvs, U):
    (sAB, nAB), (sA, nA), (sB, nB) = tvs
    
    sBA = sAB * sA / low(sB)
    nBA = nAB * nB / low(nA)
    
    return (sBA, nBA)

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

def notFormula(tvs, U):
    [(sA, nA)]  = tvs
    return (1.0 - sA, nA)

def andSymmetricFormula(tvs, U):
    total_strength = 1.0
    total_confidence = 1.0
    
    for (s, n) in tvs:
        total_strength *= s
        total_confidence *= count_to_confidence(n)
    
    return (total_strength, confidence_to_count(total_confidence))

def orFormula(tvs, U):
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

def revisionFormula(tvs, U):
    # revise two truth values
    [(s1, n1), (s2, n2)] = tvs

    s = (s1+s2)/2.0
    n = n1+n2

    return (s, n)

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
