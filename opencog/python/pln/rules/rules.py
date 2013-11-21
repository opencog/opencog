from opencog.atomspace import types, TruthValue

import pln.formulas as formulas

import math

'''Some Rules evaluate various kinds of logical links based explicitly on set membership. A set = a ConceptNode.
Other Rules calculate them heuristically, based on set probabilities and logical links.'''

BOOLEAN_LINKS = [types.AndLink, types.OrLink, types.NotLink]
FIRST_ORDER_LINKS = [types.InheritanceLink, types.SubsetLink, types.IntensionalInheritanceLink, types.SimilarityLink, types.ExtensionalSimilarityLink, types.IntensionalSimilarityLink]
HIGHER_ORDER_LINKS = [types.ImplicationLink, types.EquivalenceLink]

class Rule(object):

    def __init__ (self, outputs, inputs, formula):
        '''@outputs is one or more Trees representing the structure of Atom (usually a Link)
    to be produced by this Rule. If it's a variable then any kind of Atom
    can be produced.
    @inputs (list of Trees) specifies what kinds of Atoms
    are necessary to produce it. There should be variables used in the head
    that also appear in the goals. This means they will be the same atom.
    It's OK to reuse the same variable numbers in different Rules - they'll
    be converted to unique variables automatically.
    You can use either @formula or @tv. You specify a formula method from formulas.py;
    it will be called with the TVs of the relevant Atoms, to calculate the TruthValue for
    the resulting Atom.'''
        assert type(outputs) == list
        assert type(inputs) == list

        self._outputs = outputs
        self._inputs = inputs

        self.formula = formula
        self.name = self.__class__.__name__
        self.full_name=self.name+ ' (' +self._get_type_names(inputs)+ ' -> '
        self.full_name+= self._get_type_names(outputs)+')'

        print self.full_name

    def _get_type_names(self, templates):
        return ' '.join(template.type_name for template in templates)

    def calculate(self, input_atoms):
        '''Compute the output TV(s) based on the input atoms'''
        tvs = [atom.tv for atom in input_atoms]
        result_tvs = self.formula(tvs)
        if any((tv.mean < 0 or tv.mean > 1 or tv.count == 0) for tv in result_tvs):
            return None
        else:
            return result_tvs

    def standardize_apart_input_output(self, chainer):
        new_inputs = []
        new_outputs = []
        dic = {}

        for template in self._inputs:
            new_template = chainer.standardize_apart(template, dic)
            new_inputs.append(new_template)

        for template in self._outputs:
            new_template = chainer.standardize_apart(template, dic)
            new_outputs.append(new_template)

        created_vars = dic.values()

        return (new_inputs, new_outputs, created_vars)

# Inheritance Rules

class InversionRule(Rule):
    '''A->B entails B->A'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [B, A])],
            inputs=  [chainer.link(link_type, [A, B]),
                      A, B],
            formula= formulas.inversionFormula)

class IndependenceBasedDeductionRule(Rule):
    '''A->B, B->C entails A->C'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.deductionSimpleFormula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [B, C]),
                      A, B, C])

class DeductionRule(Rule):
    '''A->B, B->C entails A->C. Uses concept geometry.'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.deductionGeometryFormula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [B, C])])

# TODO add macro-rules for Abduction and Induction based on Deduction and Inversion
'''deduction
S is M, M is L, then S is L

induction
M is S, M is L, then S is L
invert  same    same

abduction
S is M, L is M, then S is L
        invert
'''
class InductionRule(Rule):
    '''M->S, M->L, S->L'''
    def __init__(self, chainer, link_type):
        S = chainer.new_variable()
        M = chainer.new_variable()
        L = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [S, L])],
            inputs=  [chainer.link(link_type, [M, S]),
                      chainer.link(link_type, [M, L]), S, M, L],
            formula= formulas.inductionFormula)

class AbductionRule(Rule):
    '''S is M, L is M, S->L'''
    def __init__(self, chainer, link_type):
        S = chainer.new_variable()
        M = chainer.new_variable()
        L = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [S, L])],
            inputs=  [chainer.link(link_type, [S, M]),
                      chainer.link(link_type, [L, M]), S, M, L],
            formula= formulas.inductionFormula)

# TODO implement transitiveSimilarityFormula
class TransitiveSimilarityRule(Rule):
    '''Similarity A B, Similarity B C => Similarity A C'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        link_type = types.SimilarityLink

        Rule.__init__(self,
            formula= formulas.transitiveSimilarityFormula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [B, C]),
                      A, B, C])

class ModusPonensRule(Rule):
    '''A->B, A entails B'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        notA = chainer.link(types.NotLink, [A])

        Rule.__init__(self,
            outputs= [B],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [notA, B]),
                      A],
            formula= formulas.modusPonensFormula)

class TermProbabilityRule(ModusPonensRule):
    def __init__(self, chainer):
        ModusPonensRule.__init__(self, chainer, types.InheritanceLink)

        self.formula=formulas.termProbabilityFormula

class InheritanceRule(Rule):
    '''Create a (mixed) InheritanceLink based on the SubsetLink and IntensionalInheritanceLink (based on the definition of mixed InheritanceLinks)'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(types.InheritanceLink, [A, B])],
            inputs=  [chainer.link(types.SubsetLink, [A, B]),
                      chainer.link(types.IntensionalInheritanceLink, [A, B])],
            formula= formulas.inheritanceFormula)

class SimilarityRule(Rule):
    '''SimilarityLink A B
       |A and B| / |A or B|'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(types.SimilarityLink, [A, B])],
            inputs=  [chainer.link(types.AndLink, [A, B]),
                      chainer.link(types.OrLink, [A, B])],
            formula= formulas.extensionalSimilarityFormula)

# Direct evaluation Rules

class MembershipBasedEvaluationRule(Rule):
    '''Base class for Rules that evaluate various kinds of logical links based (explicitly) on set membership. They can also be calculated based on the set probabilities and logical links.'''
    def __init__(self, chainer, member_type, output_type, formula):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs= [chainer.link(member_type, [x, A]),
                 chainer.link(member_type, [x, B])]

        Rule.__init__(self,
            formula= formula,
            outputs= [chainer.link(output_type, [A, B])],
            inputs=  inputs)

class SubsetEvaluationRule(MembershipBasedEvaluationRule):
    '''Compute Subset(A B) which is equivalent to P(x in B| x in A).'''
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer,
            member_type = types.MemberLink,
            output_type=types.SubsetLink,
            formula=formulas.subsetEvaluationFormula)

class NegatedSubsetEvaluationRule(MembershipBasedEvaluationRule):
    '''Computes P(B|NOT(A) === (Subset NOT(A) B).
       (MemberLink x NOT(B)).tv.mean = 1-(MemberLink x B).tv.mean'''
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        member_type = types.MemberLink
        output_type = types.SubsetLink
        inputs= [chainer.link(member_type, [x, A]),
                 chainer.link(member_type, [x, B])]

        notA = chainer.link(types.NotLink, [A])

        Rule.__init__(self,
            formula= formulas.negatedSubsetEvaluationFormula,
            outputs= [chainer.link(output_type, [notA, B])],
            inputs=  inputs)

class IntensionalInheritanceEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluates IntensionalInheritance(A B) from the definition.
       (Inheritance A B).tv.mean = Subset(ASSOC(A) ASSOC(B))
       ASSOC(A) is the set of x where AttractionLink(x, A)'''
    # So it's like SubsetEvaluation but using AttractionLinks instead of MemberLinks!
    # Reuses the subset formula. because intensional inheritance = subset based on intension rather than extension
    # heyyyy - wouldn't the AttractionLinks be the wrong way around here!
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer, 
            member_type = types.AttractionLink,
            output_type = types.IntensionalInheritanceLink,
            formula= formulas.subsetEvaluationFormula)

# TODO only for AND of two concepts.
# TODO the TV depends on the number of objects that do NOT satisfy AND(A B).
# This is VERY annoying
class AndEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluate And(A B) from the definition.
       |A and B| = |x in A and x in B|
       P(A and B) = |A and B| / N'''
    # count(a^b)
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer, 
            member_type = types.MemberLink,
            output_type = types.AndLink,
            formula= formulas.andEvaluationFormula)

class OrEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluate Or(A B) from the definition.
       |A or B| = |x in A or x in B|
       P(A or B) = |A or B| / universe'''
    # count(a|b)
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer, 
            member_type = types.MemberLink,
            output_type = types.OrLink,
            formula= formulas.orEvaluationFormula)

# It's more useful to calculate Subset(context, AndLink(A B))
# You could have a special subset evaluator that uses separate rules for and/or

#        inputs= [chainer.link(member_type, [x, A]),
#                 chainer.link(member_type, [x, B])]
#
# or generally, evaluate ANYTHING in subset (because you can just require that all of the premise nodes are Anded with context

# or really, try to do contextual reasoning. maybe do it for small relationships first?
# the above subset evaluation rule will 

class ExtensionalSimilarityEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluates ExtensionalSimilarity from the definition.'''
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer,
            member_type = types.MemberLink,
            output_type = types.ExtensionalSimilarityLink,
            formula= formulas.similarityEvaluationFormula)        

class IntensionalSimilarityEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluates IntensionalSimilarity from the definition.'''
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer,
            member_type = types.AttractionLink,
            output_type = types.IntensionalSimilarityLink,
            formula= formulas.similarityEvaluationFormula)        

# TODO maybe make the closed world assumption? you need plenty of 0 MemberLinks to make these calculations work right

class ExtensionalLinkEvaluationRule(Rule):
    '''Using (MemberLink x A) and (MemberLink x B), evaluate (Subset A B), (Subset B A), (Subset NOT(A) B), (Subset NOT(B) A), and (SimilarityLink A B). This is more efficient than having to find them separately using the different rules. If you use this Rule, do NOT include the separate rules too! (Or the chainer will use all of them and screw up the TV.
TODO include AndLink + OrLink too (might as well)
TODO the forward chainer will work fine with this rule, but the backward chainer won't (because it would require all of the output atoms to already exist at least with 0,0 TV), but it should probably only require one of them.'''
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs= [chainer.link(types.MemberLink, [x, A]),
                 chainer.link(types.MemberLink, [x, B])]

        outputs= [chainer.link(types.SubsetLink, [A, B]),
                  chainer.link(types.SubsetLink, [B, A]),
                  chainer.link(types.SubsetLink, [chainer.link(types.NotLink,[A]), B]),
                  chainer.link(types.SubsetLink, [chainer.link(types.NotLink,[B]), A]),
                  chainer.link(types.ExtensionalSimilarityLink, [A, B])]

        Rule.__init__(self, formula=formulas.extensionalEvaluationFormula,
            inputs=inputs,
            outputs=outputs)

class IntensionalLinkEvaluationRule(Rule):
    '''Using (AttractionLink A x) and (AttractionLink B x), evaluate (IntensionalInheritance A B), (IntensionalInheritance B A), and (IntensionalSimilarityLink A B).'''
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs= [chainer.link(types.AttractionLink, [A, x]),
                 chainer.link(types.AttractionLink, [B, x])]

        outputs= [chainer.link(types.IntensionalInheritanceLink, [A, B]),
                  chainer.link(types.IntensionalInheritanceLink, [B, A]),
                  chainer.link(types.IntensionalSimilarityLink, [A, B])]

        Rule.__init__(self, formula=formulas.extensionalEvaluationFormula,
            inputs=inputs,
            outputs=outputs)


class EvaluationToMemberRule(Rule):
    '''Turns EvaluationLink(PredicateNode P, argument) into 
       MemberLink(argument, ConceptNode "SatisfyingSet(P)".
       The argument must be a single Node.
       #The argument can either be a single Node/Link or a ListLink or arguments.'''
    def __init__(self, chainer):
        P = chainer.new_variable()
        ARG = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula= None,
                      inputs=  [chainer.link(types.EvaluationLink, [P, ARG])],
                      outputs= [])

    def custom_compute(self, inputs, outputs):
        [eval_link] = inputs
        [predicate, arg] = eval_link.out

        # Only support the case with 1 argument
        if arg.type == types.ListLink:
            if len(arg.out) == 1:
                arg = arg.out[0]
            else:
                return ([], [])

        concept_name = 'SatisfyingSet(%s)' % (predicate.name,)
        set_node = self.chainer.node(types.ConceptNode, concept_name)

        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return ([member_link], [tv])

def create_general_evaluation_to_member_rules(chainer):
    rules = []
    for argument_count in xrange(2, 3): # Bizarbitrary constant!
        for index in xrange(0, argument_count):
            rules.append(GeneralEvaluationToMemberRule(chainer, index, argument_count))
            rules.append(GeneralAtTimeEvaluationToMemberRule(chainer, index, argument_count))

    return rules

class GeneralEvaluationToMemberRule(Rule):
    '''An EvaluationLink with 2+ arguments has a satisfying set where every member is a ListLink. But there's another option which may be more useful. If you specify all but one of the arguments, you get a new predicate with only one variable left. And its satisfying set would just be normal objects.
Given (EvaluationLink pred (ListLink $thing ...)), create
sat_set =(ConceptNode "SatisfyingSet pred _ blah blah)
(MemberLink $thing sat_set)''' 
    def __init__(self, chainer, index, arg_count):
        self.index = index
        #self.arg_count= arg_count
        self.chainer = chainer

        pred = chainer.new_variable()
        all_args = chainer.make_n_variables(arg_count)
        list_link = chainer.link(types.ListLink, all_args)

        self.chainer = chainer
        Rule.__init__(self,
                      formula= None,
                      inputs=  [chainer.link(types.EvaluationLink, [pred, list_link])],
                      outputs= [])

    def custom_compute(self, inputs, outputs):
        [eval_link] = inputs
        [predicate, list_link] = eval_link.out

        args = list_link.out
        parameter_names = ['%s:%s' % (arg.name, arg.type_name) for arg in args]
        parameter_names[self.index] = '_'
        parameter_names = ' '.join(parameter_names)
        concept_name = 'SatisfyingSet(%s %s)' % (predicate.name, parameter_names)

        set_node = self.chainer.node(types.ConceptNode, concept_name)

        arg = args[self.index]
        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return ([member_link], [tv])

class GeneralAtTimeEvaluationToMemberRule(Rule):
    '''EvaluationLinks in AtTimeLinks are good for temporal rules, but don't let PLN use the normal logical rules. If you have (AtTime some_time (Evaluation near jade jades_stuff then jades_stuff is a member of the set "objects that are near jade at some time".
$x where (AtTimeLink ? EvaluationLink near jade $x)
''' 
    def __init__(self, chainer, index, arg_count):
        self.index = index
        self.chainer = chainer

        time = chainer.new_variable()
        pred = chainer.new_variable()
        all_args = chainer.make_n_variables(arg_count)
        list_link = chainer.link(types.ListLink, all_args)
        eval_link = chainer.link(types.EvaluationLink, [pred, list_link])
        at_time = chainer.link(types.AtTimeLink, [time, eval_link])

        self.chainer = chainer
        Rule.__init__(self,
                      formula= None,
                      inputs=  [at_time],
                      outputs= [])

    def custom_compute(self, inputs, outputs):
        [at_time] = inputs
        [time, eval_link] = at_time.out
        [predicate, list_link] = eval_link.out

        args = list_link.out
        parameter_names = ['%s:%s' % (arg.name, arg.type_name) for arg in args]
        parameter_names[self.index] = '_'
        parameter_names = ' '.join(parameter_names)
        concept_name = 'SatisfyingSet(sometimes %s %s)' % (predicate.name, parameter_names)

        set_node = self.chainer.node(types.ConceptNode, concept_name)

        arg = args[self.index]
        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return ([member_link], [tv])

class LinkToLinkRule(Rule):
    '''Base class for Rules that convert one link type to another'''
    def __init__(self, chainer, from_type, to_type, formula):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula= formula,
            outputs= [chainer.link(to_type, [A, B])],
            inputs=  [chainer.link(from_type, [A, B])])

class MemberToInheritanceRule(LinkToLinkRule):
    '''MemberLink(Jade robot) => MemberLink Jade {Jade}, InheritanceLink({Jade} robot).
       {Jade} is the set containing only Jade.'''
    def __init__(self, chainer):
        # use link2link rule so that backward chaining will know approximately the right target.
        LinkToLinkRule.__init__(self, chainer, from_type=types.MemberLink, to_type=types.InheritanceLink,
            formula= None)
        self.chainer=chainer

    def custom_compute(self, inputs, outputs):
        [mem_link] = inputs
        [object, superset] = mem_link.out

        singleton_concept_name = '{%s %s}' % (object.type_name, object.name,)
        singleton_set_node = self.chainer.node(types.ConceptNode, singleton_concept_name)

        member_link = self.chainer.link(types.MemberLink, [object, singleton_set_node])
        tvs = [TruthValue(1, formulas.confidence_to_count(0.99))]

        inh_link = self.chainer.link(types.InheritanceLink, [singleton_set_node, superset])
        tvs += formulas.mem2InhFormula([mem_link.tv]) # use mem2inh formula

        return ([member_link, inh_link], tvs)

# Is it a good idea to have every possible rule? Ben says no, you should bias the cognition by putting in particularly useful/synergistic rules.
#class MemberToSubsetRule(LinkToLinkRule):
#    '''MemberLink(A B) => SubsetLink(A B)'''
#    def __init__(self, chainer):
#        LinkToLinkRule.__init__(self, chainer, from_type=types.MemberLink, to_type=types.SubsetLink,
#            formula= formulas.mem2InhFormula)

class AttractionRule(Rule):
    '''Creates ExtensionalAttractionLink(A, B) <s>.
       P(Attr A B) = P(B|A) - P(B). (If it's a negative number just say 0)
       It should be P(B|Not A) rather than P(B) but that would be more expensive/annoying.'''
    def __init__(self, chainer):
        self._chainer = chainer
        A = chainer.new_variable()
        B = chainer.new_variable()

        subset1 = chainer.link(types.SubsetLink, [A, B])
        subset2 = chainer.link(types.SubsetLink, [chainer.link(types.NotLink, [A]), B])

        Rule.__init__(self,
            formula= formulas.attractionFormula,
            outputs= [chainer.link(types.AttractionLink, [A, B])],
            inputs=  [subset1, subset2])

class OntologicalInheritanceRule(Rule):
    '''Create an isa ontology.'''
    def __init__(self, chainer):
        self._chainer = chainer
        A = chainer.new_variable()
        B = chainer.new_variable()

        inhAB = chainer.link(types.InheritanceLink, [A, B])
        inhBA = chainer.link(types.InheritanceLink, [B, A])
        ontoinhAB = chainer.link(types.OntologicalInheritanceLink, [A, B])

        Rule.__init__(self,
            formula= formulas.ontoInhFormula,
            inputs=  [inhAB, inhBA],
            outputs= [ontoinhAB])

