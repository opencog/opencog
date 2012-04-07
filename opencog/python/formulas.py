from atomspace_remote import count_to_confidence, confidence_to_count

import operator, functools, itertools

from util import concat_lists

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 1.0
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 1.0
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 1.0

def identityFormula(tvs, U):
    [(sA, nA)] = tvs
    
    return (sA, nA)

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
    (sA, nA), (sB, nB) = tvs
    
    N = len(tvs)
    if N > 2:
        # TODO handle via divide-and-conquer or something
        pass
    
    A = sA * nB
    B = sB * nA
    
    s_tot = sA + sB
    n_tot = nA + nB - (A + B) / 2
    
    return (s_tot, n_tot)

def ext2InhFormula(tvs, U):
    [(sAB, nAB)] = tvs
    
    sABint = sAB
    nABint = nAB * EXTENSION_TO_INTENSION_DISCOUNT_FACTOR
    return (sABint, nABint)

def low(n):
    return max(n, 0.00001)
