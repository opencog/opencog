from opencog.atomspace import count_to_confidence, confidence_to_count, TruthValue

import operator, functools, itertools

from utility.util import concat_lists

DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 0.9
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 0.9
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 0.9
MembershipToInheritanceCountDiscountFactor = 0.9

def identityFormula(tvs):
    return tvs

def tv_seq_to_tv_tuple_seq(tvs):
    return [(tv.mean, tv.count) for tv in tvs]

# I didn't incorporate count into the formulas, it just makes things tacky.
# There are also some divide-by-zero errors where a TV is 0 (or 1, because NOT(A) is used in some formulas). If the formulas are designed well enough, that can still be compatible with indefinite TVs.

def makeUpCount(tvs):
    ArbitraryDiscountFactor = 0.9
    return min(tv.count for tv in tvs)*ArbitraryDiscountFactor

def deductionIndependenceBasedFormula(tvs):
    [(sAB, nAB), (sBC, nBC), (sB, nB),  (sC, _)] = tv_seq_to_tv_tuple_seq(tvs)

    sNotB = 1-sB

    if sNotB == 0:
        return [TruthValue(0, 0)]

    sAC = sAB*sBC  + (1-sAB)*(sC-sC*sBC)/sNotB

    n = makeUpCount(tvs)*INDEPENDENCE_ASSUMPTION_DISCOUNT

    return [TruthValue(sAC, n)]

# better deduction formula based on concept geometry.
# I swear that's not the formula in the book though
def deductionGeometryFormula(tvs):
    [AB, BC] = tvs

    sAC = AB.mean*BC.mean / min(AB.mean+BC.mean, 1)
    nAC = makeUpCount(tvs)

    return [TruthValue(sAC, nAC)]

def inversionFormula(tvs):
    [AB, A, B] = tvs
    
    sBA = AB.mean * B.mean / low(A.mean)
    nBA = makeUpCount(tvs)
    
    return [TruthValue(sBA, nBA)]

def inductionFormula(tvs):
    # InversionRule on the initial argument and then Deduction
    MS, ML, S, M, L = tvs

    [SM] = inversionFormula([MS, M, S])
    SL = deductionIndependenceBasedFormula([SM, ML, M, L])
    return SL

def abductionFormula(tvs):
    # InversionRUle on the final argument and then Deduction
    SM, LM, S, M, L = tvs

    [ML] = inversionFormula([LM, L, M])
    SL = deductionIndependenceBasedFormula([SM, ML, M, L])
    return SL

def modusPonensFormula(tvs):
    [AB, A] = tvs

    NotAB=TruthValue(0.2, 1)

    return preciseModusPonensFormula([AB, NotAB, A])

def preciseModusPonensFormula(tvs):
    (sAB, nAB), (sNotAB, _), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    sB = sAB*sA + sNotAB*negate(sA)
    
    n = makeUpCount(tvs)

    return [TruthValue(sB, n)]

def symmetricModusPonensFormula(tvs):
    (simAB, nAB), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    sNotAB = 0.2

    sB = sA*simAB + sNotAB*negate(sA)*(1+simAB)

    n = makeUpCount(tvs)

    return [TruthValue(sB, n)]

def termProbabilityFormula(tvs):
    # sB = sA*sAB/sBA
    # A, Inheritance A B, Inheritance B A => B

    [AB, BA, A] = tvs
    
    sB = A.mean*AB.mean/BA.mean
    nB = makeUpCount(tvs)

    return [TruthValue(sB, nB)]

def transitiveSimilarityFormula(tvs):
    [AB, BC, A, B, C] = tvs
    simAB = AB.mean
    simBC = BC.mean
    sA = A.mean
    sB = B.mean
    sC = C.mean

    def deduction(freqAB, freqBC):
        sAC = freqAB*freqBC + negate(freqAB)*(sC-sB*freqBC)/negate(sB)
        return sAC

    T1 = (1+sB/sA)*simAB / (1+simAB)
    T2 = (1+sC/sB)*simBC / (1+simBC)
    T3 = (1+sB/sC)*simBC / (1+simBC)
    T4 = (1+sA/sB)*simAB / (1+simAB)
    
    inhAC = deduction(T1, T2)
    inhCA = deduction(T3, T4)

    # Given inhAC and inhCA you can estimate simAC
    simAC = invert(invert(inhAC)+invert(inhCA)-1)

    count = makeUpCount(tvs)

    return [TruthValue(simAC, count)]

def inheritanceFormula(tvs):
    tv_subset, tv_inh = tvs

    # simple average of subset and inheritance
    mean = (tv_subset.mean + tv_inh.mean) /2.0
    count = makeUpCount(tvs)

    return [TruthValue(mean, count)]

def notFormula(tvs):
    [notA] = tvs
    return [TruthValue(negate(notA.mean), makeUpCount(tvs))]

def andFormula(tvs):
    total_strength = 1.0
    
    for tv in tvs:
        total_strength *= tv.mean
    
    return [TruthValue(total_strength, makeUpCount(tvs))]

def andExclusionFormula(tvs):
    [orAB, A, B] = tvs
    # Use inclusion-exclusion.
    s = A.mean+B.mean - orAB.mean

    return [TruthValue(s, makeUpCount(tvs))]

def orFormula(tvs):
    assert len(tvs) >= 2

    total_s = 1.0
    for tv in tvs[1:]:
        andAB=total_s*tv.mean
        total_s = total_s + tv.mean - andAB

    return [TruthValue(total_s, makeUpCount(tvs))]

def binaryOrFormula(tvs):
    (sA, nA), (sB, nB) = tv_seq_to_tv_tuple_seq(tvs)

    # Uses the inclusion-exclusion formula.
    andAB=sA*sB
    s = sA + sB - andAB
    n_tot = makeUpCount(tvs)
    
    return [TruthValue(s, n_tot)]

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

def twoInheritanceToSimilarityFormula(tvs):
    [(sAB, nAB), (sBA, nBA)] = tv_seq_to_tv_tuple_seq(tvs)

    s = invert( invert(sAB) + invert(sBA) -1)
    n = (nAB + nBA) / (1 + s)

    return [TruthValue(s, n)]

def oneInheritanceToSimilarityFormula(tvs):
    [(sAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)

    meat = (sA/sC+1)/sAC - 1

    simAC = invert(meat)

    return [TruthValue(simAC, makeUpCount(tvs))]  

def similarityToInheritanceFormula(tvs):
    '''Given simAB, sA and sB, estimate sAB. Could easily be turned around to
       estimate sBA'''
    [(simAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)

    sAB = (1+sB/sA)*simAB / (1 + simAB)

    return [TruthValue(sAB, makeUpCount(tvs))]

def mem2InhFormula(tvs):
    [mem_tv] = tvs
    count = mem_tv.count * MembershipToInheritanceCountDiscountFactor

    return [TruthValue(mem_tv.mean, count)]

inh2MemFormula = mem2InhFormula

def fuzzy_and(mean0, mean1):
    return min(mean0, mean1)

def fuzzy_or(mean0, mean1):
    return max(mean0, mean1)

def subsetEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

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

def andEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

    # P(x in B AND x in A)
    and_ab = mem_a and mem_b

    if and_ab:
        # This object is in A and B
        return [TruthValue(1, 1)]
    else:
        # This object is not in A AND B
        # So raise the count by one, lowering the probability slightly
        return [TruthValue(0, 1)]

def orEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

    # P(x in B OR x in A)
    or_ab = mem_a or mem_b

    if or_ab:
        return [TruthValue(1, 1)]
    else:
        return [TruthValue(0, 1)]

def negatedSubsetEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_not_a_mean = 1 - mem_a_tv.mean
    mem_not_a_tv = TruthValue(mem_not_a_mean, mem_a_tv.count)

    return subsetEvaluationFormula([mem_not_a_tv, mem_b_tv])

def subsetFuzzyEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs

    fuzzy_and_ab = fuzzy_and(mem_a_tv.mean, mem_b_tv.mean)

    # P(x in B | x in A) = P(x in A ^ x in B) / P(x in A)

    # ... TODO implement this formula. my head got sore --Jade

def similarityEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

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
Outputs: SubsetLink A B.tv, SubsetLink B A.tv, SubsetLink NOT(A) B.tv, SubsetLink NOT(B) A.tv SimilarityLink A B.tv'''
    subsetAB = subsetEvaluationFormula(tvs)
    subsetBA = subsetEvaluationFormula(reversed(tvs))
    subsetNotAB = negatedSubsetEvaluationFormula(tvs)
    subsetNotBA = negatedSubsetEvaluationFormula(reversed(tvs))

    similarityAB = similarityEvaluationFormula(tvs)

    # Each of those formulas returns a list containing one TV, and this formula returns a list containing 3 TVs
    tvs = subsetAB + subsetBA + subsetNotAB + subsetNotBA + similarityAB
    return tvs

def extensionalSimilarityFormula(tvs):
    [and_tv, or_tv] = tvs

    # calculate the size of the two sets A AND B vs A OR B
    and_size = 1.0*and_tv.mean*and_tv.count
    or_size  = 1.0*or_tv.mean*or_tv.count

    if or_size == 0:
        return [TruthValue(0, 0)]

    P = and_size / or_size
    N = makeUpCount(tvs)

    return [TruthValue(P, N)]

def attractionFormula(tvs):
    [ab, b] = tvs

    s = low(ab.mean - b.mean)

    count = makeUpCount(tvs)

    return [TruthValue(s, count)]

def ontoInhFormula(tvs):
    [ab, ba] = tvs

    mean = low(ab.mean - ba.mean)

    count = makeUpCount(tvs)

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
    nB = makeUpCount(tvs)

    return [TruthValue(sB, nB)]

def orBreakdownFormula(tvs):
    [A, OR_AB] = tvs

    sNotA = (1-A.mean)

    sB = OR_AB.mean / sNotA
    nB = makeUpCount(tvs)

    return [TruthValue(sB, nB)]

'''
Evaluation is_American Ben <fuzzy tv 1>
Implication is_American is_idiot <strength tv 2>
|-
Evaluation is_idiot Ben <tv3>

Use Mem2InhFormula to get the tv of: Implication is_Ben is_American
Use DeductionFormula to get the tv of: Implication is_ben is_idiot
Use I2M to get the tv of: Evaluation is_idiot Ben
'''
def evaluationImplicationFormula(tvs):
    [eval_B_A, impl_B_C, B, C] = tvs

    [impl_A_B] = mem2InhFormula([eval_B_A])
    [impl_A_C] = deductionIndependenceBasedFormula(
        [impl_A_B, impl_B_C, B, C])
    [eval_C_A] = inh2MemFormula([impl_A_C])

    return [eval_C_A]

def low(n):
    return max(n, 0)

def invert(n):
    return 1.0/n

def negate(n):
    return 1.0-n

