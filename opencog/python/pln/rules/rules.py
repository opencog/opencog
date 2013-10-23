from opencog.atomspace import types, TruthValue

import pln.formulas as formulas

import math

'''Some Rules evaluate various kinds of logical links based explicitly on set membership. A set = a ConceptNode.
Other Rules calculate them heuristically, based on set probabilities and logical links.'''

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
    
        for atom in self._inputs + self._outputs:
            assert atom.type != 65535 # missing type bug (cython issue?)

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

        return (new_inputs, new_outputs)

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

class DeductionRule(Rule):
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

# TODO add macro-rules for Abduction and Induction based on Deduction and Inversion
# abandoned
class InductionRule(Rule):
    '''A->B, A->C entails B->C'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [B, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [A, C])],
            formula= None)
            
    def compute(inputs):
        pass

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

        Rule.__init__(self,
            outputs= [B],
            inputs=  [chainer.link(link_type, [A, B]),
                      A],
            formula= formulas.modusPonensFormula)

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

# Boolean link creation Rules
# An EliminationRule uses a logical link to produce its arguments

class NotCreationRule(Rule):
    '''A => NotLink(A)'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [chainer.link(types.NotLink, [A])],
            inputs= [A])

class NotEliminationRule(Rule):
    '''NotLink(A) => A'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [A],
            inputs=  [chainer.link(types.NotLink, [A])])

def make_n_variables(chainer, N):
    return [chainer.new_variable() for i in xrange(0, N)]

# TODO These should take account of dependencies in some cases

def create_and_or_rules(chainer, min_n, max_n):
    rules = []
    for n in min_n, max_n:
        rules.append(AndCreationRule(chainer, n))
        rules.append(OrCreationRule(chainer, n))
        rules.append(AndEliminationRule(chainer, n))
        rules.append(OrEliminationRule(chainer, n))

    return rules

class AndCreationRule(Rule):
    '''Take a set of N atoms and create AndLink(atoms)'''
    def __init__(self, chainer, N):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= formulas.andSymmetricFormula,
            outputs= [chainer.link(types.AndLink, atoms)],
            inputs=  atoms)

class OrCreationRule(Rule):
    '''[A, B...] => Or(A, B...)'''
    def __init__(self, chainer, N):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= formulas.orFormula,
            outputs= [chainer.link(types.OrLink, atoms)],
            inputs=  atoms)

# Elimination Rules

class AbstractEliminationRule(Rule):
    def __init__(self, chainer, N, link_type):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= None,
            outputs= atoms,
            inputs=  [chainer.link(link_type, atoms)])

class AndEliminationRule(AbstractEliminationRule):
    '''AndLink(atoms) => atoms'''
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self, chainer, N, link_type= types.AndLink)

    def calculate(self, atoms):
        [and_atom] = atoms
        outputs = and_atom.out
        N = len(outputs)

        # assume independence, i.e. P(A^B^C...) = P(A)P(B)P(C)...
        # therefore P(A) = Nth root of P(AndLink)
        # same for P(B) etc
        individual_frequency = math.pow(and_atom.tv.mean, 1.0/N)
        individual_count = and_atom.tv.count/1.42

        output_tvs = [TruthValue(individual_frequency, individual_count) for out in outputs]

        return output_tvs

class OrEliminationRule(AbstractEliminationRule):
    '''Take OrLink(atoms) and produce all of the atoms separately'''
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self, chainer, N, link_type= types.OrLink)

    def calculate(self, atoms):
        [or_atom] = atoms
        outputs = or_atom.out
        N = len(outputs)

        # TODO this formula is wrong: it assumes P(A or B or C...) = P(A)+P(B)+P(C)...
        # therefore P(A) = P(OrLink)/N
        # same for P(B) etc
        individual_mean = or_atom.tv.mean/N
        count = 1 # hack

        output_tvs = [TruthValue(individual_mean, count) for out in outputs]

        return output_tvs

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

class IntensionalInheritanceEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluates IntensionalInheritance(A B) from the definition.
       (Inheritance A B).tv.mean = Subset(ASSOC(A) ASSOC(B))
       ASSOC(A) is the set of x where AttractionLink(x, A)'''
    # So it's like SubsetEvaluation but using AttractionLinks instead of MemberLinks!
    # Reuses the subset formula. because intensional inheritance = subset based on intension rather than extension
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(self, chainer, 
            member_type = types.AttractionLink,
            output_type = types.IntensionalInheritanceLink,
            formula= formulas.subsetEvaluationFormula)

# abandoned for now because you have to estimate the number of objects (maybe an arbitrary setting?)
class AndEvaluationRule(MembershipBasedEvaluationRule):
    '''Evaluate And(A B) from the definition.
       |A and B| = |x in A and x in B|
       P(A and B) = |A and B| / universe (gulp)'''
    pass

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

class EvaluationToMemberRule(Rule):
    '''Turns EvaluationLink(PredicateNode P, argument) into 
       MemberLink(argument, ConceptNode "SatisfyingSet(P)".
       The argument can either be a single Node/Link or a ListLink or arguments.'''
    def __init__(self, chainer):
        P = chainer.new_variable()
        ARG = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula= None,
                      inputs=  [chainer.link(types.EvaluationLink, [P, ARG])],
                      outputs= [])

    def custom_compute(self, inputs):
        [eval_link] = inputs
        [predicate, arg] = eval_link.out

        concept_name = 'SatisfyingSet(%s)' % (predicate.name,)
        set_node = self.chainer.node(types.ConceptNode, concept_name)

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
    '''MemberLink(Ben American) => MemberLink Ben {Ben}, InheritanceLink({Ben} American).
       {Ben} is the set containing only Ben.'''
    def __init__(self, chainer):
        # use link2link rule so that backward chaining will know approximately the right target.
        LinkToLinkRule.__init__(self, chainer, from_type=types.MemberLink, to_type=types.InheritanceLink,
            formula= None)

    def custom_compute(self, inputs):
        [mem_link] = inputs
        [object, superset] = mem_link.out

        singleton_concept_name = '{%s %s}' % (object.type_name, object.name,)
        singleton_set_node = self.chainer.node(types.ConceptNode, concept_name)

        member_link = self.chainer.link(types.MemberLink, [object, singleton_set_node])
        tvs = [TruthValue(1, formulas.confidence_to_count(1))]

        self.chainer.link(types.InheritanceLink, [singleton_set_node, superset])
        tvs += formulas.mem2InhFormula([mem_link]) # use mem2inh formula

        return ([member_link], tvs)

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

        Rule.__init__(self,
            formula= formulas.attractionFormula,
            outputs= [chainer.link(types.AttractionLink, [A, B])],
            inputs=  [chainer.link(types.SubsetLink, [A, B]),
                      B])

# redundant now
class ASSOCEvaluationRule(Rule):
    '''Creates the extensional association set of ConceptNode C (called ASSOC_ext(C).
    MemberLink(e, ASSOC_ext(C)).tv = Func(Subset(e, C), Subset(Not(e), C)).

    Or now that we have AttractionLink
    MemberLink(e, ASSOC(C)).tv = AttractionLink e C'''
    pass

