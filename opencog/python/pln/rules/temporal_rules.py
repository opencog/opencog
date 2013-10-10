from opencog.atomspace import types, TruthValue

import pln.formulas
import pln.temporal_formulas

import math

class TemporalRule(Rule):
    '''Base class for temporal Rules. They evaluate a time relation such as BeforeLink, based on some AtTimeLinks.'''
    # TODO it can't use the formulas directly. They require some preprocessing to create a temporal-distribution-dictionary out of some AtTimeLinks. And how to find all of the AtTimeLinks for an Atom?
    # Easy way: the inputs here will eventually find every element of A and B, so just recalculate every time.
    def __init__(self, chainer, link_type, formula):
        A = chainer.new_variable()
        B = chainer.new_variable()
        ta = chainer.new_variable()
        tb = chainer.new_variable()

        Rule.__init__(self,
            formula= formula,
            outputs= [chainer.link(link_type, [A, B])],
            inputs=  [chainer.link(types.AtTimeLink(ta, A)),
                      chainer.link(types.AtTimeLink(tb, B))]

        name = link_type + 'EvaluationRule'

    def temporal_compute(self, input_tuples):
        links_a, links_b = unzip(input_tuples)

        dist1 = self.make_distribution(links_a)
        dist2 = self.make_distribution(links_b)

        strength = formula(dist1, dist2)
        
        # I'm not sure what to choose for this
        count = len(input_tuples)
        tv = TruthValue(strength, count)
        
        return [(target,tv)]

    def make_distribution(self, time_links):
        dist = {}
        for link in time_links:
            time = int(link.out[0].name)
            fuzzy_tv = link.tv.mean
            dist[time] = fuzzy_tv
        
        return dist


class TemporalTransitivityRule(Rule):
    # Hackily infer transitive temporal relationships using the deduction formula
    # This Rule is important but the TV formula is wrong
    def __init__(self, chainer, link_type, formula= formulas.deductionFormula):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self, chainer, formula=formula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]), chainer.link(link_type, [B, C])])

        name = link_type + 'TransitivityRule'

def create_temporal_rules(chainer):
    rules = []

    # directly evaluate temporal links
    combinations = [
        (types.BeforeLink, temporal_formulas.beforeFormula),
        (types.OverlapsLink, temporal_formulas.overlapsFormula),
        (types.DuringLink, temporalFormulas.duringFormula),
        (types.MeetsLink, temporalFormulas.meetsFormula),
        (types.StartsLink, temporalFormulas.startsFormula),
        (types.FinishesLink, temporalFormulas.finishesFormula),
        (types.EqualsLink, temporalFormulas.equalsFormula),
        ]

    for (type, formula) in combinations:
        rules.append(TemporalRule(chainer, type, formula))

    for type, _ in combinations:
        # use temporal links to evaluate more temporal links
        rules.append(TemporalTransitivityRule(chainer, type))

    return rules 

    # There are lots of reverse links, (like (After x y) = (Before y x)
    # It seems like those would just make it dumber though?

