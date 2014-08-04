from opencog.atomspace import TruthValue
from math import isinf, isnan

# Todo: Could this file be broken up so that there would be a
# one-to-one correspondence between 'formulas' and 'rules' files?

# Todo: How should these values be chosen properly?
DEDUCTION_TERM_WEIGHT = 1.0
INDEPENDENCE_ASSUMPTION_DISCOUNT = 0.9
EXTENSION_TO_INTENSION_DISCOUNT_FACTOR = 0.9
INTENSION_TO_EXTENSION_DISCOUNT_FACTOR = 0.9
MembershipToInheritanceCountDiscountFactor = 0.9
REVISION_REDUNDANCY_FACTOR = 0.5


def identityFormula(tvs):
    return tvs


def tv_seq_to_tv_tuple_seq(tvs):
    return [(tv.mean, tv.count) for tv in tvs]

# Todo: What needs to be done in regards to the following comment?
# I didn't incorporate count into the formulas, it just makes things
# tacky. There are also some divide-by-zero errors where a TV is 0
# (or 1, because NOT(A) is used in some formulas). If the formulas
# are designed well enough, that can still be compatible with
# indefinite TVs.


# Todo: Which of the following is correct?
def makeUpCount(tvs):
    #ArbitraryDiscountFactor = 0.9
    #return min(tv.count for tv in tvs)*ArbitraryDiscountFactor
    return min(tv.count for tv in tvs)


def deductionIndependenceBasedFormula(tvs):
    [(sAB, nAB), (sBC, nBC), (sB, nB),  (sC, _)] = tv_seq_to_tv_tuple_seq(tvs)

    sNotB = 1 - sB

    if sNotB == 0:
        return [TruthValue(0, 0)]

    sAC = sAB * sBC + (1 - sAB) * (sC - sB * sBC) / sNotB

    n = makeUpCount(tvs) * INDEPENDENCE_ASSUMPTION_DISCOUNT

    assert not isinf(sAC)
    assert not isnan(sAC)
    assert not isinf(n)
    assert not isnan(n)

    return [TruthValue(sAC, n)]


# Todo: Check the correctness of this formula.
# better deduction formula based on concept geometry.
# I swear that's not the formula in the book though
def deductionGeometryFormula(tvs):
    [AB, BC] = tvs

    sAC = AB.mean * BC.mean / min(AB.mean + BC.mean, 1)
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
    # InversionRule on the final argument and then Deduction
    SM, LM, S, M, L = tvs

    [ML] = inversionFormula([LM, L, M])
    SL = deductionIndependenceBasedFormula([SM, ML, M, L])
    return SL


def modusPonensFormula(tvs):
    [AB, A] = tvs

    # Todo: what should these strength and count values actually be?
    # see bug: https://github.com/opencog/opencog/issues/598
    NotAB = TruthValue(0.2, TruthValue().confidence_to_count(1))

    return preciseModusPonensFormula([AB, NotAB, A])


def preciseModusPonensFormula(tvs):
    (sAB, nAB), (sNotAB, _), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    sB = sAB * sA + sNotAB * negate(sA)
    
    n = makeUpCount(tvs)

    return [TruthValue(sB, n)]


def symmetricModusPonensFormula(tvs):
    (simAB, nAB), (sA, nA) = tv_seq_to_tv_tuple_seq(tvs)

    sNotAB = 0.2

    sB = sA * simAB + sNotAB * negate(sA) * (1 + simAB)

    n = makeUpCount(tvs)

    return [TruthValue(sB, n)]


def termProbabilityFormula(tvs):
    # sB = sA*sAB/sBA
    # A, Inheritance A B, Inheritance B A => B

    [AB, BA, A] = tvs
    
    sB = A.mean * AB.mean / BA.mean
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
        sAC = freqAB * freqBC + negate(freqAB) * (sC - sB * freqBC) / negate(sB)
        return sAC

    T1 = (1 + sB / sA) * simAB / (1 + simAB)
    T2 = (1 + sC / sB) * simBC / (1 + simBC)
    T3 = (1 + sB / sC) * simBC / (1 + simBC)
    T4 = (1 + sA / sB) * simAB / (1 + simAB)
    
    inhAC = deduction(T1, T2)
    inhCA = deduction(T3, T4)

    # Given inhAC and inhCA you can estimate simAC
    simAC = invert(invert(inhAC) + invert(inhCA) - 1)

    count = makeUpCount(tvs)

    return [TruthValue(simAC, count)]


def inheritanceFormula(tvs):
    tv_subset, tv_inh = tvs

    # simple average of subset and inheritance
    mean = (tv_subset.mean + tv_inh.mean) / 2.0
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
        andAB = total_s * tv.mean
        total_s = total_s + tv.mean - andAB

    return [TruthValue(total_s, makeUpCount(tvs))]


def binaryOrFormula(tvs):
    (sA, nA), (sB, nB) = tv_seq_to_tv_tuple_seq(tvs)

    # Uses the inclusion-exclusion formula.
    andAB = sA * sB
    s = sA + sB - andAB
    n_tot = makeUpCount(tvs)
    
    return [TruthValue(s, n_tot)]


# Todo: parameter 'U' value is not used
def andPartitionFormula(tvs, U):
    [(sAndA, nAndA), (sAndB, nAndB)] = tvs

    s = sAndA * sAndB
    n = nAndA + nAndB
    return [TruthValue(s, n)]


# Todo: parameter 'U' value is not used
def ext2InhFormula(tvs, U):
    [(sAB, nAB)] = tvs
    
    sABint = sAB
    nABint = nAB * EXTENSION_TO_INTENSION_DISCOUNT_FACTOR
    return [TruthValue(sABint, nABint)]

# Todo: This is a duplicate definition. The symbol "inheritanceFormula"
# already exists in this file. Which one should be used?
# def inheritanceFormula(tvs, U):
#     [(sExt, nExt), (sInt, nInt)] = tvs
#
#     s = (sExt + sInt) / 2.0
#     n = (nExt + nInt) / 2.0
#
#     return [TruthValue(s, n)]


def twoInheritanceToSimilarityFormula(tvs):
    [(sAB, nAB), (sBA, nBA)] = tv_seq_to_tv_tuple_seq(tvs)

    s = invert(invert(sAB) + invert(sBA) - 1)
    n = (nAB + nBA) / (1 + s)

    return [TruthValue(s, n)]


# Todo: this formula contains unresolved references to 'sC' and 'sAC',
# so it currently doesn't work
def oneInheritanceToSimilarityFormula(tvs):
    [(sAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)

    meat = (sA / sC + 1) / sAC - 1

    simAC = invert(meat)

    return [TruthValue(simAC, makeUpCount(tvs))]  


def similarityToInheritanceFormula(tvs):
    """
    Given simAB, sA and sB, estimate sAB. Could easily be turned
    around to estimate sBA
    """
    [(simAB, nAB), (sA, nA), (sB, nB)] = tv_seq_to_tv_tuple_seq(tvs)

    sAB = (1 + sB / sA) * simAB / (1 + simAB)

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

        # Todo: why does the count become 1?
        return [TruthValue(1, 1)]
    else:
        # A and NOTB => 1 observation of NOTB|A

        # Todo: why does the count become 1?
        return [TruthValue(0, 1)]


def andEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

    # P(x in B AND x in A)
    and_ab = mem_a and mem_b

    if and_ab:
        # This object is in A and B

        # Todo: why does the count become 1?
        return [TruthValue(1, 1)]
    else:
        # This object is not in A AND B
        # So raise the count by one, lowering the probability slightly

        # Todo: why does the count become 1?
        return [TruthValue(0, 1)]


def orEvaluationFormula(tvs):
    [mem_a_tv, mem_b_tv] = tvs
    mem_a = mem_a_tv.mean >= 0.5
    mem_b = mem_b_tv.mean >= 0.5

    # P(x in B OR x in A)
    or_ab = mem_a or mem_b

    if or_ab:
        # Todo: why does the count become 1?
        return [TruthValue(1, 1)]
    else:
        # Todo: why does the count become 1?
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

        # Todo: why does the count become 1?
        return [TruthValue(1, 1)]
    else:
        # increment |A or B| without changing |A and B|

        # Todo: why does the count become 1?
        return [TruthValue(0, 1)]


def extensionalEvaluationFormula(tvs):
    """
    Inputs:
        Membership x A.tv
        Membership x B.tv

    Outputs:
        SubsetLink A B.tv
        SubsetLink B A.tv
        SubsetLink NOT(A) B.tv
        SubsetLink NOT(B) A.tv
        SimilarityLink A B.tv
    """
    subsetAB = subsetEvaluationFormula(tvs)
    subsetBA = subsetEvaluationFormula(reversed(tvs))
    subsetNotAB = negatedSubsetEvaluationFormula(tvs)
    subsetNotBA = negatedSubsetEvaluationFormula(reversed(tvs))

    similarityAB = similarityEvaluationFormula(tvs)

    # Each of those formulas returns a list containing one TV, and
    # this formula returns a list containing 3 TVs
    tvs = subsetAB + subsetBA + subsetNotAB + subsetNotBA + similarityAB
    return tvs


def intensionalEvaluationFormula(tvs):
    """
    Inputs:
        Attraction x A.tv
        Attraction x B.tv

    Outputs:
        IntensionalInheritance A B.tv
        IntensionalInheritance B A.tv
        IntensionalSimilarityLink A B.tv
    """
    subsetAB = subsetEvaluationFormula(tvs)
    subsetBA = subsetEvaluationFormula(reversed(tvs))

    similarityAB = similarityEvaluationFormula(tvs)

    # Each of those formulas returns a list containing one TV, and this
    # formula returns a list containing 3 TVs
    tvs = subsetAB + subsetBA + similarityAB
    return tvs


def extensionalSimilarityFormula(tvs):
    [and_tv, or_tv] = tvs

    # calculate the size of the two sets A AND B vs A OR B
    and_size = 1.0 * and_tv.mean * and_tv.count
    or_size = 1.0 * or_tv.mean * or_tv.count

    if or_size == 0:
        return [TruthValue(0, 0)]

    P = and_size / or_size
    N = makeUpCount(tvs)

    return [TruthValue(P, N)]

def subsetFormula(tvs):
    [and_tv, a_tv] = tvs

    sAB = and_tv.mean / a_tv.mean
    nAB = makeUpCount(tvs)

    return [TruthValue(sAB, nAB)]

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
    """
    From the PLN Book: We may also heuristically form a count rule
    such as n = n1 + n2 - c min(n1, n2), where the parameter c indicates an 
    assumption about the level of interdependency between the bases of 
    information corresponding to the two premises. The value c=1 denotes 
    the assumption that the two sets of evidence are completely redundant; 
    the value c=0 denotes the assumption that the two sets of evidence are 
    totally distinct. Intermediate values denote intermediate assumptions.
    """
    x, y = tvs
    # revise two truth values

    #n = x.count+y.count
    n = x.count + y.count - REVISION_REDUNDANCY_FACTOR * min(x.count, y.count)
    
    weight_1 = x.count * 1.0 / n
    weight_2 = y.count * 1.0 / n
    # TODO maybe check for overlap
    #s = (weight_1 * x.mean + weight_2 * y.mean)

    #TODO: formula temporarily changed, due to this issue:
    #   https://github.com/opencog/opencog/issues/646
    s = (x.count * x.mean + y.count * y.mean) / (x.count + y.count)

    return TruthValue(s, n)


def andBreakdownFormula(tvs):
    [A, AND_AB] = tvs

    sB = AND_AB.mean / A.mean
    nB = makeUpCount(tvs)

    return [TruthValue(sB, nB)]


def orBreakdownFormula(tvs):
    [A, OR_AB] = tvs

    sNotA = (1 - A.mean)

    sB = OR_AB.mean / sNotA
    nB = makeUpCount(tvs)

    return [TruthValue(sB, nB)]

# Todo: What needs to be done here?
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


def andAs1stArgInsideLinkFormula(tvs):
    """
    tv of InheritanceLink (AndLink A B) C =
    P(C | A) * P(A) * P(C | B) * P(B) / (P(C) * P(A&B))

    @see: https://github.com/opencog/opencog/pull/904

    :param tvs: tvs of InheritanceLink A B,
                       InheritanceLink B C,
                       A, B, C
    :return: tvs of InheritanceLink (AndLink A B) C, AndLink A B
    """
    inh_ac_tv, inh_bc_tv, a_tv, b_tv, c_tv = tvs

    and_ab_tv = andFormula([a_tv, b_tv])[0]
    inh_and_ab_c_strength = (inh_ac_tv.mean * a_tv.mean * inh_bc_tv.mean
                             * b_tv.mean) / (c_tv.mean * and_ab_tv.mean)
    inh_and_ab_c_count = makeUpCount(tvs)

    return [TruthValue(inh_and_ab_c_strength, inh_and_ab_c_count), and_ab_tv]


def andAs2ndArgInsideLinkFormula(tvs):
    """
    tv of InheritanceLink A (AndLink B C) =
    P(B | A) * P(A) * P(C | A) * P(B&C) / (P(B) * P(C))

    :param tvs: tvs of InheritanceLink A B,
                       InheritanceLink A C,
                       A, B, C
    :return: tvs of InheritanceLink A (AndLink B C), AndLink B C
    """
    inh_ab_tv, inh_ac_tv, a_tv, b_tv, c_tv = tvs

    and_bc_tv = andFormula([b_tv, c_tv])[0]
    inh_a_and_bc_strength = (inh_ab_tv.mean * a_tv.mean * inh_ac_tv.mean
                             * and_bc_tv.mean) / (b_tv.mean * c_tv.mean)
    inh_a_and_bc_count = makeUpCount(tvs)

    return [TruthValue(inh_a_and_bc_strength, inh_a_and_bc_count), and_bc_tv]


def contextFormula(tvs):
    return tvs

def low(n):
    return max(n, 0)


def invert(n):
    return 1.0 / n


def negate(n):
    return 1.0 - n
