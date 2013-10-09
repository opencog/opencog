from opencog.atomspace import types, TruthValue

import pln.formulas
import pln.temporal_formulas

import math

class TemporalRule(Rule):
    '''Base class for temporal Rules. They evaluate a time relation such as BeforeLink, based on some AtTimeLinks.'''
    # TODO it can't use the formulas directly. They require some preprocessing to create a temporal-distribution-dictionary out of some AtTimeLinks. And how to find all of the AtTimeLinks for an Atom?...
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


# old hack that might not be relevant to the new chainer

def lookup_times(atom, chainer):
    '''Search for all AtTimeLinks containing that atom. Return a dictionary from timestamp (as an int) to a float, representing the fuzzy truth value. (Ignore confidence which is probably OK.)'''
    template = chainer.link(types.AtTimeLink,
                 chainer.new_variable(),
                 atom)
    candidate_atoms = chainer.atomspace.get_atoms_by_type(types.AtTimeLink)
    attimes = chainer.find(template, candidate_atoms)
    
    dist = {}
    for link in attimes:
        time = int(link.out[0].name)
        fuzzy_tv = link.tv.mean
        dist[time] = fuzzy_tv
    
    return dist

def create_temporal_matching_function(formula):
    def match_temporal_relationship(space,target):
        
        distribution_event1 = lookup_times(target.args[0], space)
        distribution_event2 = lookup_times(target.args[1], space)
        
        missing_data = (len(distribution_event1) == 0 or len(distribution_event2) == 0)
        error_message = "unable to find distribution for targets", target.args[0], distribution_event1, target.args[1], distribution_event2

        #assert not missing_data, error_message
        # Enable this instead of the assert after you finish debugging the rules.
        # For real-world use the assert is wrong - if you don't have the right data you should just not apply that Rule.
        if missing_data:
            print error_message
            # No info available for those events. So return no results
            return []

        strength = formula(distribution_event1, distribution_event2)
        
        # I'm not sure what to choose for this
        count = 1
        tv = TruthValue(strength, count)
        
        return [(target,tv)]

    return match_temporal_relationship

