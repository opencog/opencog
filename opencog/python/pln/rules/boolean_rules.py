import math
from opencog.atomspace import types, TruthValue
from pln.rule import Rule

# Heuristically create boolean links using the TruthValues of their arguments

# TODO These should take account of dependencies in some cases
from pln.rules import formulas


def create_and_or_rules(chainer, min_n, max_n):
    rules = []
    for n in xrange(min_n, max_n):
        rules.append(AndCreationRule(chainer, n))
        rules.append(OrCreationRule(chainer, n))
        rules.append(AndEliminationRule(chainer, n))
        rules.append(OrEliminationRule(chainer, n))

    rules.append(AndBreakdownRule(chainer))
    rules.append(OrBreakdownRule(chainer))

    rules.append(NotCreationRule(chainer))
    rules.append(NotEliminationRule(chainer))

    return rules


class BooleanLinkCreationRule(Rule):
    # TODO has bugs so i disabled it
    # e.g. it will convert And(And(A B) And(B C)) into And(A B B C) which is redundant
    def disabled_custom_compute(self, inputs, outputs):
        # the output may have a hierarchy of links, but we should
        # flatten it (this means PLN can incrementally create bigger
        # boolean links)
        output_tvs = self.calculate(inputs)
        if output_tvs is None:
            # Formula created invalid TV
            return [], []

        actual_output = simplify_boolean(self._chainer, outputs[0])

        return [actual_output], output_tvs


class NotCreationRule(BooleanLinkCreationRule):
    """
    A => NotLink(A)
    """
    def __init__(self, chainer):
        A = chainer.new_variable()
        self._chainer = chainer

        Rule.__init__(self,
                      formula=formulas.notFormula,
                      outputs=[chainer.link(types.NotLink, [A])],
                      inputs=[A])


#TODO: some heuristic is necessary when this rule is applied so that AndLinks
#aren't produced arbitrarily; otherwise AndLinks of AndLinks will also be
#produced
class AndCreationRule(BooleanLinkCreationRule):
    """
    Take a set of N atoms and create AndLink(atoms)
    """
    def __init__(self, chainer, N):
        atoms = chainer.make_n_variables(N)
        self._chainer = chainer

        Rule.__init__(self,
                      formula=formulas.andFormula,
                      outputs=[chainer.link(types.AndLink, atoms)],
                      inputs=atoms,
                      name = "AndCreationRule<"+str(N)+">")


class OrCreationRule(BooleanLinkCreationRule):
    '''[A, B...] => Or(A, B...)'''
    def __init__(self, chainer, N):
        atoms = chainer.make_n_variables(N)
        self._chainer = chainer

        Rule.__init__(self,
                      formula=formulas.orFormula,
                      outputs=[chainer.link(types.OrLink, atoms)],
                      inputs=atoms,
                      name = "OrCreationRule<"+str(N)+">")


def simplify_boolean(chainer, link):
    """
    This function can be called after each inference to simplify the
    resulting expression.
    """
    # Not(Not(A)) => A
    if link.type == types.NotLink:
        arg = link.out[0]
        if arg.type == types.NotLink:
            deeply_nested_arg = arg.out[0]
            return deeply_nested_arg
        return link

    # And(A And(B, C) D) => And(A, B, C, D)
    elif link.type == types.AndLink:
        new_out = []
        for atom in link.out:
            # AndLink containing another AndLink
            if atom.type == types.AndLink:
                new_out += atom.out
            # OrLink or ConceptNode
            else:
                new_out.append(atom)
        return chainer.link(types.AndLink, new_out)

    # likewise for OrLink
    elif link.type == types.OrLink:
        new_out = []
        for atom in link.out:
            # OrLink containing another OrLink
            if atom.type == types.OrLink:
                new_out += atom.out
            # AndLink or ConceptNode
            else:
                new_out.append(atom)
        return chainer.link(types.OrLink, new_out)

    else:
        return link


def create_boolean_transformation_rules(chainer):
    # P OR Q = Not(P) => Q
    # P AND Q = Not(P => Not(Q))
    # P <=> Q = P=>Q ^ Q => P

    rules = []

    def make_symmetric_rule(lhs, rhs):
        assert len(lhs) == 1
        assert len(rhs) == 1

        rule = Rule(inputs=lhs, outputs=rhs, formula=formulas.identityFormula)
        rule.name = 'BooleanTransformationRule<%s,%s>' % (lhs[0].type_name, rhs[0].type_name)
        rule._compute_full_name()
        rules.append(rule)

        rule = Rule(inputs=rhs, outputs=lhs, formula=formulas.identityFormula)
        rule.name = 'BooleanTransformationRule<%s,%s>' % (lhs[0].type_name, rhs[0].type_name)
        rule._compute_full_name()
        rules.append(rule)

    P, Q = chainer.make_n_variables(2)
    LHS = [chainer.link(types.OrLink, [P, Q])]
    RHS = [chainer.link(types.SubsetLink,
                        [chainer.link(types.NotLink, [P]), Q])]

    make_symmetric_rule(LHS, RHS)

    LHS = [chainer.link(types.AndLink, [P, Q])]
    RHS = [chainer.link(types.NotLink,
                        [chainer.link(types.SubsetLink,
                                      [P, chainer.link(types.NotLink, [Q])])])]

    make_symmetric_rule(LHS, RHS)

    LHS = [chainer.link(types.ExtensionalSimilarityLink, [P, Q])]
    RHS = [chainer.link(types.AndLink,
                        [chainer.link(types.SubsetLink, [P, Q]),
                        chainer.link(types.SubsetLink, [Q, P])])]

    make_symmetric_rule(LHS, RHS)

    return rules


class AbstractEliminationRule(Rule):
    def __init__(self, chainer, N, link_type):
        atoms = chainer.make_n_variables(N)

        Rule.__init__(self,
                      formula=None,
                      outputs=atoms,
                      inputs=[chainer.link(link_type, atoms)])

# Todo: explain the following comment:
# (these rules are generally bad approximations)


class AndBreakdownRule(AbstractEliminationRule):
    """
    A, (AndLink A B) => B
    """
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      formula=formulas.andBreakdownFormula,
                      outputs=[B],
                      inputs=[A, chainer.link(types.AndLink, [A, B])])


class OrBreakdownRule(AbstractEliminationRule):
    """
    A, (OrLink A B) => B
    """
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      formula=formulas.orBreakdownFormula,
                      outputs=[B],
                      inputs=[A, chainer.link(types.OrLink, [A, B])])

# Todo: explain the following comment:
# Very hacky Elimination Rules

class AndEliminationRule(AbstractEliminationRule):
    """
    AndLink(atoms) => atoms
    """
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self,
                                         chainer,
                                         N,
                                         link_type=types.AndLink)

    def calculate(self, atoms):
        [and_atom] = atoms
        outputs = and_atom.out
        N = len(outputs)

        # assume independence, i.e. P(A^B^C...) = P(A)P(B)P(C)...
        # therefore P(A) = Nth root of P(AndLink)
        # same for P(B) etc
        individual_frequency = math.pow(and_atom.tv.mean, 1.0/N)
        individual_count = and_atom.tv.count/1.42

        # Todo: The variable 'out' is not used
        output_tvs = [TruthValue(individual_frequency, individual_count)
                      for out in outputs]

        return output_tvs


class OrEliminationRule(AbstractEliminationRule):
    """
    Take OrLink(atoms) and produce all of the atoms separately
    """
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self,
                                         chainer,
                                         N,
                                         link_type=types.OrLink)

    def calculate(self, atoms):
        [or_atom] = atoms
        outputs = or_atom.out
        N = len(outputs)

        # TODO this formula is wrong:
        # it assumes P(A or B or C...) = P(A)+P(B)+P(C)...
        # therefore P(A) = P(OrLink)/N
        # same for P(B) etc
        individual_mean = or_atom.tv.mean/N
        # Todo:
        count = 1  # hack

        # Todo: The variable 'out' is not used
        output_tvs = [TruthValue(individual_mean, count) for out in outputs]

        return output_tvs


class NotEliminationRule(Rule):
    '''NotLink(A) => A'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
                      formula=formulas.notFormula,
                      outputs=[A],
                      inputs=[chainer.link(types.NotLink, [A])])


class AndBulkEvaluationRule(Rule):
    """
    Bulk evaluate And(A B) based on MemberLinks. Unlike
    AndEvaluationRule this will find every MemberLink at the same time
    (much more efficient than doing it online at random).
    It autodetects whether the AndLink has ConceptNodes or EvaluationLinks.
    If ConceptNodes, it will search for Concepts that are members of the given concepts.
    If PredicateNodes, it will search for tuples (i.e. ListLinks) that have an EvaluationLink with the given predicate.
    If EvaluationLinks, they should all have the same number of arguments which should all just be variablenodes. (Less general but a lot simpler)
    """
    def __init__(self, chainer, N):
        self._chainer = chainer

        vars = chainer.make_n_variables(N)

        Rule.__init__(self,
                      name="AndBulkEvaluationRule<%s>"%(N,),
                      formula=None,
                      outputs=[chainer.link(types.AndLink, vars)],
                      inputs=[])

    def custom_compute(self, inputs, outputs):
        # It must only be used in backward chaining. The inputs will be
        # [] and the outputs will be [And(conceptNode123, conceptNode456)]
        # or similar. It uses the Python set class and won't work with
        # variables.
        [and_link_target] = outputs
        and_args = and_link_target.out
        if any(atom.is_a(types.VariableNode) for atom in and_args):
            return [], []

        # The backward chainer will find weird subgoals that don't make sense
        if any(not(arg.is_a(types.ConceptNode) or arg.is_a(types.PredicateNode) or arg.is_a(types.EvaluationLink)) for arg in and_args):
            return [], []

        # An AndLink can either contain ConceptNodes, in which case we would use MemberLinks, or it can contain EvaluationLinks (in which case we would use instances of that EvaluationLink)
        if and_args[0].is_a(types.ConceptNode):
            conceptnodes = and_args
            sets = [self.get_member_links(node) for node in conceptnodes]
        elif and_args[0].is_a(types.PredicateNode):
            predicatenodes = and_args
            sets = [self.get_eval_links(node) for node in predicatenodes]
        elif and_args[0].is_a(types.EvaluationLink):
            eval_links = and_args
            predicatenodes = [link.out[0] for link in eval_links]
            sets = [self.get_eval_links(node) for node in predicatenodes]
        else:
            raise "not implemented yet"

        # filter links with fuzzy strength > 0.5 and select just the nodes
        for i in xrange(0, len(sets)):
            filteredSet = set(self.get_member(link) for link in sets[i] if link.tv.mean > 0.5)
            sets[i] = filteredSet

        intersection = sets[0]
        for i in xrange(1, len(sets)):
            intersection = intersection & sets[i]

        union = sets[0]
        for i in xrange(1, len(sets)):
            union = union | sets[i]

        nIntersection = float(len(intersection))
        nUnion = float(len(union))

        sAnd = nIntersection / nUnion
        #and_link = self._chainer.link(types.AndLink, and_args)
        and_link = outputs[0]

        nAnd = nUnion

        return [and_link], [TruthValue(sAnd, nAnd)]

    def get_member_links(self, conceptnode):
        return set(self._chainer.find_members(conceptnode))

    def get_member(self, link):
        if link.is_a(types.MemberLink):
            return link.out[0]
        else:
            return link.out[1]

    def get_eval_links(self, predicatenode):
        return set(self._chainer.find_eval_links(predicatenode))

class NegatedAndBulkEvaluationRule(AndBulkEvaluationRule):
    """
    Bulk evaluate And(Not(And(A B C)) D). It only handles EvaluationLinks.
    It is evaluating the probability of D in things that don't satisfy And(A B C). (It can be used with AndToSubsetRule1 to create Implication(Not(And A B C) D).
    So once you also have Implication(Not(And A B C) D) you can use PreciseModusPonensRule to find P(D|A,B,C)!
    """
    def __init__(self, chainer, N):
        self._chainer = chainer

        vars = chainer.make_n_variables(N)
        notlink = chainer.link(types.NotLink,
                    [chainer.link(types.AndLink, vars[0:-1])])
        andlink = chainer.link(types.AndLink, [notlink, vars[-1]])

        Rule.__init__(self,
                      name="NegatedAndBulkEvaluationRule<%s>"%(N,),
                      formula=None,
                      outputs=[andlink],
                      inputs=[])

    def custom_compute(self, inputs, outputs):
        # It must only be used in backward chaining. The inputs will be
        # [] and the outputs will be [And(conceptNode123, conceptNode456)]
        # or similar. It uses the Python set class and won't work with
        # variables.
        [and_link_target] = outputs
        top_and_args = and_link_target.out

        #import pdb; pdb.set_trace()

        # You can't assume that the first link is the NotLink because AndLink is an UnorderedLink and the AtomSpace will choose a canonical order
        if not top_and_args[0].is_a(types.NotLink):
            top_and_args.reverse()

        not_link = top_and_args[0]
        negated_args = not_link.out[0].out        

        other_eval_link = top_and_args[1]

        if any(atom.is_a(types.VariableNode) for atom in negated_args+[other_eval_link]):
            return [], []

        if not negated_args[0].is_a(types.EvaluationLink):
            assert "not implemented yet"

        eval_links = negated_args+[other_eval_link]
        predicatenodes = [link.out[0] for link in eval_links]
        sets = [self.get_eval_links(node) for node in predicatenodes]

        # filter links with fuzzy strength > 0.5 and select just the nodes
        for i in xrange(0, len(sets)):
            filteredSet = set(self.get_member(link) for link in sets[i] if link.tv.mean > 0.5)
            sets[i] = filteredSet

        # Find the intersection of the things in the NotLink (A&B&C)
        # Then find things that are in D but not (A&B&C) = D-(A&B&C)
        intersection = sets[0]
        for i in xrange(1, len(sets)-1):
            intersection = intersection & sets[i]

        intersection= sets[-1] - intersection

        union = sets[0]
        for i in xrange(1, len(sets)):
            union = union | sets[i]

        nIntersection = float(len(intersection))
        nUnion = float(len(union))

        sAnd = nIntersection / nUnion
        #and_link = self._chainer.link(types.AndLink, and_args)
        and_link = outputs[0]

        nAnd = nUnion

        #print and_link, TruthValue(sAnd, nAnd)

        return [and_link], [TruthValue(sAnd, nAnd)]

