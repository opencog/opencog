from opencog.atomspace import types, TruthValue
import formulas
from pln.rule import Rule

'''
Some Rules evaluate various kinds of logical links based explicitly on
set membership. A set = a ConceptNode. Other Rules calculate them
heuristically, based on set probabilities and logical links.
'''

# Todo: try to separate these rules further into several files by
# category. The rules in this file were under the header 'direct
# evaluation rules' in rules.py, but may need to be further classified.

__VERBOSE__ = False

BOOLEAN_LINKS = [types.AndLink,
                 types.OrLink,
                 types.NotLink]

FIRST_ORDER_LINKS = [types.InheritanceLink,
                     types.SubsetLink,
                     types.IntensionalInheritanceLink,
                     types.SimilarityLink,
                     types.ExtensionalSimilarityLink,
                     types.IntensionalSimilarityLink]

HIGHER_ORDER_LINKS = [types.ImplicationLink,
                      types.EquivalenceLink]


class MembershipBasedEvaluationRule(Rule):
    """
    Base class for Rules that evaluate various kinds of logical links
    based (explicitly) on set membership. They can also be calculated
    based on the set probabilities and logical links.
    """
    def __init__(self, chainer, member_type, output_type, formula):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs = [chainer.link(member_type, [x, A]),
                  chainer.link(member_type, [x, B])]

        Rule.__init__(self,
                      formula=formula,
                      outputs=[chainer.link(output_type, [A, B])],
                      inputs=inputs)

        self.probabilistic_inputs = False


class SubsetEvaluationRule(MembershipBasedEvaluationRule):
    """
    Compute Subset(A B) which is equivalent to P(x in B| x in A).
    """
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.MemberLink,
            output_type=types.SubsetLink,
            formula=formulas.subsetEvaluationFormula)


class NegatedSubsetEvaluationRule(MembershipBasedEvaluationRule):
    """
    Computes P(B|NOT(A) === (Subset NOT(A) B).
    (MemberLink x NOT(B)).tv.mean = 1-(MemberLink x B).tv.mean
    """
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        member_type = types.MemberLink
        output_type = types.SubsetLink
        inputs = [chainer.link(member_type, [x, A]),
                  chainer.link(member_type, [x, B])]

        notA = chainer.link(types.NotLink, [A])

        Rule.__init__(self,
                      formula=formulas.negatedSubsetEvaluationFormula,
                      outputs=[chainer.link(output_type, [notA, B])],
                      inputs=inputs)

        self.probabilistic_inputs = False


class IntensionalInheritanceEvaluationRule(MembershipBasedEvaluationRule):
    """
    Evaluates IntensionalInheritance(A B) from the definition.
    (Inheritance A B).tv.mean = Subset(ASSOC(A) ASSOC(B))
    ASSOC(A) is the set of x where AttractionLink(x, A)
    """
    # So it's like SubsetEvaluation but using AttractionLinks instead
    # of MemberLinks! Reuses the subset formula. because intensional
    # inheritance = subset based on intension rather than extension
    # heyyyy - wouldn't the AttractionLinks be the wrong way around here!
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.AttractionLink,
            output_type=types.IntensionalInheritanceLink,
            formula=formulas.subsetEvaluationFormula)


# TODO only for AND of two concepts.
# TODO the TV depends on the number of objects that do NOT satisfy AND(A B).
# This is VERY annoying
class AndEvaluationRule(MembershipBasedEvaluationRule):
    """
    Evaluate And(A B) from the definition.
    |A and B| = |x in A and x in B|
    P(A and B) = |A and B| / N
    """
    # count(a^b)
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.MemberLink,
            output_type=types.AndLink,
            formula=formulas.andEvaluationFormula)


class OrEvaluationRule(MembershipBasedEvaluationRule):
    """
    Evaluate Or(A B) from the definition.
    |A or B| = |x in A or x in B|
    P(A or B) = |A or B| / universe
    """
    # count(a|b)
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.MemberLink,
            output_type=types.OrLink,
            formula=formulas.orEvaluationFormula)

# Todo: What needs to be done in regards to the following comment?
# It's more useful to calculate Subset(context, AndLink(A B))
# You could have a special subset evaluator that uses separate rules
# for and/or
#
#        inputs= [chainer.link(member_type, [x, A]),
#                 chainer.link(member_type, [x, B])]
#
# or generally, evaluate ANYTHING in subset (because you can just
# require that all of the premise nodes are Anded with context or
# really, try to do contextual reasoning. maybe do it for small
# relationships first? the above subset evaluation rule will


class ExtensionalSimilarityEvaluationRule(MembershipBasedEvaluationRule):
    """
    Evaluates ExtensionalSimilarity from the definition.
    """
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.MemberLink,
            output_type=types.ExtensionalSimilarityLink,
            formula=formulas.similarityEvaluationFormula)


class IntensionalSimilarityEvaluationRule(MembershipBasedEvaluationRule):
    """
    Evaluates IntensionalSimilarity from the definition.
    """
    def __init__(self, chainer):
        MembershipBasedEvaluationRule.__init__(
            self, chainer,
            member_type=types.AttractionLink,
            output_type=types.IntensionalSimilarityLink,
            formula=formulas.similarityEvaluationFormula)

# TODO maybe make the closed world assumption? you need plenty of 0
# MemberLinks to make these calculations work right


class ExtensionalLinkEvaluationRule(Rule):
    """
    Using (MemberLink x A) and (MemberLink x B), evaluate (Subset A B),
    (Subset B A), (Subset NOT(A) B), (Subset NOT(B) A), and
    (SimilarityLink A B). This is more efficient than having to find
    them separately using the different rules. If you use this Rule, do
    NOT include the separate rules too! (Or the chainer will use all of
    them and screw up the TV.

    TODO include AndLink + OrLink too (might as well)
    TODO the forward chainer will work fine with this rule, but the
    backward chainer won't (because it would require all of the output
    atoms to already exist at least with 0,0 TV), but it should
    probably only require one of them.
    """
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs = [chainer.link(types.MemberLink, [x, A]),
                  chainer.link(types.MemberLink, [x, B])]

        outputs = [chainer.link(types.SubsetLink, [A, B]),
                   chainer.link(types.SubsetLink, [B, A]),
                   chainer.link(types.SubsetLink,
                                [chainer.link(types.NotLink, [A]), B]),
                   chainer.link(types.SubsetLink,
                                [chainer.link(types.NotLink, [B]), A]),
                   chainer.link(types.ExtensionalSimilarityLink, [A, B])]

        Rule.__init__(self,
                      formula=formulas.extensionalEvaluationFormula,
                      inputs=inputs,
                      outputs=outputs)

        self.probabilistic_inputs = False


class IntensionalLinkEvaluationRule(Rule):
    """
    Using (AttractionLink A x) and (AttractionLink B x), evaluate
    (IntensionalInheritance A B), (IntensionalInheritance B A), and
    (IntensionalSimilarityLink A B).
    """
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs = [chainer.link(types.AttractionLink, [x, A]),
                  chainer.link(types.AttractionLink, [x, B])]

        outputs = [chainer.link(types.IntensionalInheritanceLink, [A, B]),
                   chainer.link(types.IntensionalInheritanceLink, [B, A]),
                   chainer.link(types.IntensionalSimilarityLink, [A, B])]

        Rule.__init__(
            self,
            formula=formulas.intensionalEvaluationFormula,
            inputs=inputs,
            outputs=outputs)


class EvaluationToMemberRule(Rule):
    """
    Turns EvaluationLink(PredicateNode Predicate:Z, argument) into
    MemberLink(argument, ConceptNode:Z).
    The argument must be a single Node.
    """
    def __init__(self, chainer):
        Z = chainer.new_variable()
        ARG = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.EvaluationLink, [Z, ARG])],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [eval_link] = inputs
        [predicate, arg] = eval_link.out

        # Only support the case with 1 argument
        if arg.type == types.ListLink:
            if len(arg.out) == 1:
                arg = arg.out[0]
            else:
                return [], []

        concept_name = predicate.name
        set_node = self.chainer.node(types.ConceptNode, concept_name)

        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return [member_link], [tv]


class MemberToEvaluationRule(Rule):
    """
    Turns MemberLink(argument, ConceptNode:Z) into
    EvaluationLink(PredicateNode Predicate:Z, argument).
    The argument must be a single Node.
    """
    def __init__(self, chainer):
        Z = chainer.new_variable()
        ARG = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.MemberLink, [ARG, Z])],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [member_link] = inputs
        [arg, concept] = member_link.out

        predicate_node = self.chainer.node(types.PredicateNode, concept.name)

        evaluation_link = self.chainer.link(types.EvaluationLink,
                             [predicate_node,
                             self.chainer.link(types.ListLink, [arg])])
        tv = member_link.tv

        return [evaluation_link], [tv]


# Todo: Should this function be in this file with the rules?
def create_general_evaluation_to_member_rules(chainer):
    rules = []
    # Todo: What needs to be done to the following constants?
    for argument_count in xrange(2, 3):  # Bizarbitrary constant!
        for index in xrange(0, argument_count):
            rules.append(GeneralEvaluationToMemberRule(chainer,
                                                       index,
                                                       argument_count))
            rules.append(GeneralAtTimeEvaluationToMemberRule(chainer,
                                                             index,
                                                             argument_count))

    return rules


class GeneralEvaluationToMemberRule(Rule):
    """
    An EvaluationLink with 2+ arguments has a satisfying set where
    every member is a ListLink. But there's another option which may be
    more useful. If you specify all but one of the arguments, you get a
    new predicate with only one variable left. And its satisfying set
    would just be normal objects. Given:
      (EvaluationLink pred (ListLink $thing ...))
    create:
      sat_set = (ConceptNode "SatisfyingSet pred _ blah blah)
      (MemberLink $thing sat_set)
    """
    def __init__(self, chainer, index, arg_count):
        self.index = index
        #self.arg_count= arg_count
        self.chainer = chainer

        pred = chainer.new_variable()
        all_args = chainer.make_n_variables(arg_count)
        list_link = chainer.link(types.ListLink, all_args)

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.EvaluationLink,
                                           [pred, list_link])],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [eval_link] = inputs
        [predicate, list_link] = eval_link.out

        args = list_link.out
#        parameter_names = ['%s:%s' % (arg.name, arg.type_name) for arg in args]
#        parameter_names[self.index] = '_'
#        parameter_names = ' '.join(parameter_names)
#        concept_name = 'SatisfyingSet(%s %s)' % (predicate.name,
#                                                 parameter_names)

#        set_node = self.chainer.node(types.ConceptNode, concept_name)

#        arg = args[self.index]
#        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        concept_1 = args[0]
        concept_2 = args[1]
        x = self.chainer.node(types.VariableNode, "$X")
        eval_list_link = self.chainer.link(types.ListLink, [x, concept_2])
        evaluation_link = self.chainer.link(types.EvaluationLink, [predicate , eval_list_link])
        satisying_set_link = self.chainer.link(types.SatisfyingSetLink, [x, evaluation_link])
        member_link = self.chainer.link(types.MemberLink, [concept_1, satisying_set_link])

        return [member_link], [tv]


class GeneralAtTimeEvaluationToMemberRule(Rule):
    """
    EvaluationLinks in AtTimeLinks are good for temporal rules, but
    don't let PLN use the normal logical rules. If you have
    (AtTime some_time (Evaluation near jade jades_stuff))
    then jades_stuff is a member of the set "objects that are near
    jade at some time".
    $x where (AtTimeLink ? EvaluationLink near jade $x)
    """
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
                      formula=None,
                      inputs=[at_time],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [at_time] = inputs
        # Todo: The 'time' variable is never used
        [time, eval_link] = at_time.out
        [predicate, list_link] = eval_link.out

        args = list_link.out
        parameter_names = ['%s:%s' % (arg.name, arg.type_name) for arg in args]
        parameter_names[self.index] = '_'
        parameter_names = ' '.join(parameter_names)
        concept_name = 'SatisfyingSet(sometimes %s %s)' % \
                       (predicate.name, parameter_names)

        set_node = self.chainer.node(types.ConceptNode, concept_name)

        arg = args[self.index]
        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return [member_link], [tv]


class LinkToLinkRule(Rule):
    """
    Base class for Rules that convert one link type to another
    """
    def __init__(self, chainer, from_type, to_type, formula):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula=formula,
            outputs=[chainer.link(to_type, [A, B])],
            inputs=[chainer.link(from_type, [A, B])])


class MemberToInheritanceRule(LinkToLinkRule):
    """
    MemberLink(Jade robot) => InheritanceLink(Jade robot).
    """
    def __init__(self, chainer):
        LinkToLinkRule.__init__(self,
                                chainer,
                                from_type=types.MemberLink,
                                to_type=types.InheritanceLink,
                                formula=formulas.mem2InhFormula)
        self.chainer = chainer

        self.probabilistic_inputs = False


class InheritanceToMemberRule(LinkToLinkRule):
    """
    InheritanceLink(Jade robot) => MemberLink(Jade robot).
    """
    def __init__(self, chainer):
        LinkToLinkRule.__init__(self,
                                chainer,
                                from_type=types.InheritanceLink,
                                to_type=types.MemberLink,
                                formula= formulas.mem2InhFormula)
        self.chainer = chainer

# Is it a good idea to have every possible rule? Ben says no, you
# should bias the cognition by putting in particularly useful/
# synergistic rules.

# Todo: this rule was commented out
#class MemberToSubsetRule(LinkToLinkRule):
#    '''MemberLink(A B) => SubsetLink(A B)'''
#    def __init__(self, chainer):
#        LinkToLinkRule.__init__(self,
#                                chainer,
#                                from_type=types.MemberLink,
#                                to_type=types.SubsetLink,
#                                formula= formulas.mem2InhFormula)


class AttractionRule(Rule):
    """
    Creates ExtensionalAttractionLink(A, B) <s>.
    P(Attr A B) = P(B|A) - P(B|Not A). (If it's a negative number just say 0)
    """
    def __init__(self, chainer):
        self._chainer = chainer
        A = chainer.new_variable()
        B = chainer.new_variable()

        subset1 = chainer.link(types.SubsetLink, [A, B])
        subset2 = chainer.link(types.SubsetLink,
                               [chainer.link(types.NotLink, [A]), B])

        Rule.__init__(self,
                      formula=formulas.attractionFormula,
                      outputs=[chainer.link(types.AttractionLink, [A, B])],
                      inputs=[subset1, subset2])


class OntologicalInheritanceRule(Rule):
    """
    Create an isa ontology.
    """
    def __init__(self, chainer):
        self._chainer = chainer
        A = chainer.new_variable()
        B = chainer.new_variable()

        inhAB = chainer.link(types.InheritanceLink, [A, B])
        inhBA = chainer.link(types.InheritanceLink, [B, A])
        ontoinhAB = chainer.link(types.OntologicalInheritanceLink, [A, B])

        Rule.__init__(self,
            formula=formulas.ontoInhFormula,
            inputs=[inhAB, inhBA],
            outputs=[ontoinhAB])


class ProcedureEvaluationRule(Rule):
    """
    Evaluate EvaluationLinks.
    EvaluationLink (PredicateNode F) (ListLink arguments).
    """
    def __init__(self, chainer):
        self._chainer = chainer
        F = chainer.new_variable()
        args = chainer.new_variable()

        evallink = chainer.link(types.EvaluationLink, [F, args])

        Rule.__init__(self,
                      formula=None,
                      inputs=[],
                      outputs=[evallink])

    # Todo: Should this method be static, or access a class variable?
    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs=None):
        [eval_link] = inputs
        [function, list_link] = eval_link.out

        function.name
        args = list_link.out

        # Todo: What is this for?
        # assume it's a python function
        def is_jade(arguments):
            node = arguments[0]
            if node.is_a(types.ConceptNode) and node.name == 'jade':
                return 1.0

        fuzzy_tv = is_jade(args)

        # Todo: Why is the count 1?
        return [TruthValue(fuzzy_tv, 1.0)]
