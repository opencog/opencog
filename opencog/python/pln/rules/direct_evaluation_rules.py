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


class MemberToEvaluationRule(Rule):
    """
    Turns inputs formated as
     MemberLink
         $Y
         SatisfyingSet
              $X
              EvaluationLink($X)
    to EvaluationLink ($Y).
    Can handle multiple argument EvaluationLinks.
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
        evaluation_link = []
        tv = member_link.tv

        if member_link.out[1].type == types.SatisfyingSetLink:
            concept = member_link.out[0]
            variable = member_link.out[1].out[0]
            predicate = member_link.out[1].out[1].out[0]
            input_args = member_link.out[1].out[1].out[1].out
            output_args = [concept if i == variable else i for i in input_args]

            list_link = self.chainer.link(types.ListLink, output_args)
            evaluation_link = [self.chainer.link(types.EvaluationLink,
                                 [predicate, list_link])]

        return evaluation_link, [tv]


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
        self.arg_count= arg_count
        self.chainer = chainer

        pred = chainer.new_variable()
        all_args = chainer.make_n_variables(arg_count)
        list_link = chainer.link(types.ListLink, all_args)

        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.EvaluationLink,
                                           [pred, list_link])],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [eval_link] = inputs
        [predicate, arg] = eval_link.out
        variables = [self.chainer.node(types.VariableNode, "$X{}".format(i))
                     for i in xrange(0, self.arg_count)]
        returned_outputs = []
        tv = []

        # arg_indexes holds the occurrence count of a particular atom in a
        # ListLink
        # Key = the atom under consideration
        # Values = the index of the atom in the ListLink
        arg_indexs = dict(((j, [p for p, q in enumerate(arg.out) if q == j])
                           for i, j in enumerate(arg.out)))

        if arg.type == types.ListLink:
            for i in arg.out:
                # The arg.out is a list that must not be changed. If arg.out is
                # used instead of the returned value of atomspace's get_outgoing
                # method then the changes made to arg.out are permanent.
                list_arg = self.chainer.atomspace.get_outgoing(arg.h)
                first_iter = True

                for j in variables:
                    if first_iter:
                        list_arg[arg_indexs[i].pop(0)] = j
                        first_iter = False
                    else:
                        try:
                            next_index = next(k for k, l in enumerate(list_arg)
                                              if l not in variables)
                        except StopIteration:
                            break
                        list_arg[next_index] = j
                    list_link = self.chainer.link(
                        types.ListLink, list_arg)
                    evaluation_link = self.chainer.link(
                        types.EvaluationLink,
                        [predicate, list_link])
                    satisfying_set_link = self.chainer.atomspace.add_link(
                        types.SatisfyingSetLink,
                        [variables[0], evaluation_link],
                        TruthValue(1, TruthValue().confidence_to_count(1)))
                    member_link = self.chainer.link(
                        types.MemberLink,
                        [i, satisfying_set_link])

                    returned_outputs.append(member_link)
                    tv.append(eval_link.tv)

        return returned_outputs, tv


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


class SatisfyingSetToConceptRule(Rule):
    """
    Given:
    (SatisfyingSetLink
        (VariableNode "$X")
        (EvaluationLink
            (PredicateNode "smokes")
            (ListLink
                (VariableNode "$X")))))
    produces:
    (AndLink
    (ConceptToPredicateLink
        (ConceptNode "smoke")
        (PredicateNode "smoke")
    (InheritanceLink
        (ConceptNode "smokes")
        (SatisfyingSetLink
            (VariableNode ("$X")
            (EvaluationLink
                (PredicateNode "smokes")
                (ListLink
                    (VariableNode "$X")))))
    while
    (ConceptToPredicateLink
        (ConceptNode "smoke")
        (PredicateNode "smoke"))
    is equivalent to
    (EvaluationLink
        (PredicateNode "ConceptToPredicate")
        (ListLink
            (ConceptNode "smoke")
            (PredicateNode "smoke")))
    """
    def __init__(self, chainer, list_link_arg_count):
        self.chainer = chainer
        variable = chainer.new_variable()
        predicate = chainer.new_variable()
        list_link_args = chainer.make_n_variables(list_link_arg_count)
        list_link = chainer.link(types.ListLink, list_link_args)
        eval_link = chainer.link(types.EvaluationLink, [predicate, list_link])

        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.SatisfyingSetLink,
                                           [variable, eval_link])],
                      outputs=[])

        self.probabilistic_inputs = False

    # Todo: The 'outputs' parameter is not used
    def custom_compute(self, inputs, outputs):
        [satisfying_set_link] = inputs
        and_link = []
        tv = satisfying_set_link.tv

        if satisfying_set_link.out[0].type == types.VariableNode:
            predicate = satisfying_set_link.out[1].out[0]
            new_concept = self.chainer.atomspace.add_node(types.ConceptNode,
                                                          predicate.name)
            concept_to_predicate = self.chainer.atomspace.add_node(
                types.PredicateNode, "ConceptToPredicate")
            list_link = self.chainer.link(types.ListLink,
                                          [new_concept, predicate])
            eval_link = self.chainer.link(
                types.EvaluationLink, [concept_to_predicate, list_link])
            inheritance_link = self.chainer.link(
                types.InheritanceLink, [new_concept, satisfying_set_link])
            and_link = self.chainer.link(
                types.AndLink, [eval_link, inheritance_link])

        return [and_link], [tv]
