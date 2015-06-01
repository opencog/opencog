from opencog.atomspace import types, TruthValue
from math import isinf, isnan

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


class Rule(object):
    def __init__(self, outputs, inputs, formula, name=None):
        """
        @outputs is one or more Trees representing the structure of
        Atom (usually a Link) to be produced by this Rule. If it's a
        variable then any kind of Atom can be produced.

        @inputs (list of Trees) specifies what kinds of Atoms are
        necessary to produce it. There should be variables used in the
        head that also appear in the goals. This means they will be the
        same atom. It's OK to reuse the same variable numbers in
        different Rules - they'll be converted to unique variables
        automatically. You can use either @formula or @tv. You specify
        a formula method from formulas.py; it will be called with the
        TVs of the relevant Atoms, to calculate the TruthValue for the
        resulting Atom.
        """
        assert type(outputs) == list
        assert type(inputs) == list

        self._outputs = outputs
        self._inputs = inputs

        self.formula = formula
        if name:
            self.name = name
        else:
            self.name = self.__class__.__name__

        # It's easier to just generate descriptive names (see inheritance_rules.py for example)
        self._compute_full_name()

        self.probabilistic_inputs = True

    def _compute_full_name(self):
        """
        Compute the full name based on the name and the input/output types for the rule
        """
        self.full_name = \
            self.name + ' (' + self._get_type_names(self._inputs) + ' -> '
        self.full_name += self._get_type_names(self._outputs) + ')'        

    @staticmethod
    def _get_type_names(templates):
        return ' '.join(template.type_name for template in templates)

    def __str__(self):
        return self.name

    def calculate(self, input_atoms):
        """
        Compute the output TV(s) based on the input atoms
        """
        tvs = [atom.tv for atom in input_atoms]
        try:
            result_tvs = self.formula(tvs)
            if any((tv.mean < 0 or tv.mean > 1 or tv.count == 0
               or isinf(tv.mean) or isnan(tv.mean) or isinf(tv.count) or isnan(tv.count))
                   for tv in result_tvs) or len(result_tvs) == 0:
                return None
            else:
                return result_tvs
        except ZeroDivisionError:
            return None

    def standardize_apart_input_output(self, chainer):
        chainer.log.debug("Standardizing apart for {0}".format(self.name))

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

        chainer.log.debug("New inputs:\n{0}".format(new_inputs))
        chainer.log.debug("New outputs:\n{0}".format(new_outputs))

        return new_inputs, new_outputs, created_vars

    # This uses the actual inputs chosen by the chainer
    def valid_inputs(self, inputs):
        """
        Takes all of the inputs found so far, and tells you if they are
        valid. e.g. for OrCreationRule, [A] or [A,B] are valid, but
        [A,A] is invalid because Or(A,A) is silly.
        """
        # Don't allow the same input to be used twice.
        # This function is called incrementally, so you only need to
        # test the last input on the list.
        atom = inputs[-1]
        if atom in inputs[:-1]: return False
        if atom.type in FIRST_ORDER_LINKS or atom.type in HIGHER_ORDER_LINKS:
            # Reject (Subset A A)
            self_link = atom.out[0] == atom.out[1]
            if self_link:
                return False
            # Reject [(Subset A B), (Subset B A)]
            # Reject [(Subset A B), (Subset B B)]
        return True
