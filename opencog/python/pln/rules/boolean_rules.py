from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.rules.rules as rules
Rule = rules.Rule

import math

# Heuristically create boolean links using the TruthValues of their arguments

class NotCreationRule(Rule):
    '''A => NotLink(A)'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [chainer.link(types.NotLink, [A])],
            inputs= [A])

# TODO These should take account of dependencies in some cases

def create_and_or_rules(chainer, min_n, max_n):
    rules = []
    for n in min_n, max_n:
        rules.append(AndCreationRule(chainer, n))
        rules.append(OrCreationRule(chainer, n))
        rules.append(AndEliminationRule(chainer, n))
        rules.append(OrEliminationRule(chainer, n))

    rules.append(AndBreakdownRule(chainer))
    rules.append(OrBreakdownRule(chainer))

    rules.append(NotCreationRule(chainer))
    rules.append(NotEliminationRule(chainer))

    return rules

class AndCreationRule(Rule):
    '''Take a set of N atoms and create AndLink(atoms)'''
    def __init__(self, chainer, N):
        atoms = chainer.make_n_variables(N)

        Rule.__init__(self,
            formula= formulas.andSymmetricFormula,
            outputs= [chainer.link(types.AndLink, atoms)],
            inputs=  atoms)

class OrCreationRule(Rule):
    '''[A, B...] => Or(A, B...)'''
    def __init__(self, chainer, N):
        atoms = chainer.make_n_variables(N)

        Rule.__init__(self,
            formula= formulas.orFormula,
            outputs= [chainer.link(types.OrLink, atoms)],
            inputs=  atoms)

def simplify_boolean(chainer, link):
    if link.type == types.NotLink:
        arg = link.out[0]
        if arg.type == types.NotLink:
            deeply_nested_arg = arg.out[0]
            return chainer.link(types.NotLink, [deeply_nested_arg])

    elif link.type == types.AndLink:
        new_out = []
        for atom in link.out:
            # AndLink containing another AndLink
            if atom.type == types.AndLink:
                new_out+= atom.out
            # OrLink or ConceptNode
            else:
                new_out.append(atom)

    elif link.type == types.OrLink:
        new_out = []
        for atom in link.out:
            # OrLink containing another OrLink
            if atom.type == types.OrLink:
                new_out+= atom.out
            # AndLink or ConceptNode
            else:
                new_out.append(atom)

    else:
        return link

class AbstractEliminationRule(Rule):
    def __init__(self, chainer, N, link_type):
        atoms = chainer.make_n_variables(N)

        Rule.__init__(self,
            formula= None,
            outputs= atoms,
            inputs=  [chainer.link(link_type, atoms)])

# (these rules are generally bad approximations)

class AndBreakdownRule(AbstractEliminationRule):
    '''A, (AndLink A B) => B'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula=formulas.andBreakdownFormula,
            outputs= [B],
            inputs= [A, chainer.link(types.AndLink, [A, B])])

class OrBreakdownRule(AbstractEliminationRule):
    '''A, (OrLink A B) => B'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula=formulas.orBreakdownFormula,
            outputs= [B],
            inputs= [A, chainer.link(types.OrLink, [A, B])])

# Very hacky Elimination Rules

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

class NotEliminationRule(Rule):
    '''NotLink(A) => A'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [A],
            inputs=  [chainer.link(types.NotLink, [A])])


