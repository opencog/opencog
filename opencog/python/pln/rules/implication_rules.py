from opencog.atomspace import types, TruthValue
import formulas
from pln.rule import Rule

class ImplicationAndRule(Rule):
    """
    If A  then B
    If (And B C ) then D
    |-
    If (And A C) then D
    """
    def __init__(self, chainer):
        self.chainer = chainer

        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()
        d = chainer.new_variable()
        input_and_link = chainer.link(types.AndLink, [b, c])
        output_and_link = chainer.link(types.AndLink, [a, c])
        input_impl_link_1 = chainer.link(types.ImplicationLink, [a, b])
        input_impl_link_2 = chainer.link(types.ImplicationLink, [input_and_link, d])
        output_impl_link = chainer.link(types.ImplicationLink, [output_and_link, d])

        Rule.__init__(self,
                      formula=formulas.implicationAndRuleFormula,
                      inputs=[input_impl_link_1, input_impl_link_2],
                      outputs=[output_impl_link])

        # TODO : add a custom_compute and use simplify_boolean function from 
        # boolean_rules so as to simplify the outputs when there are nested AndLinks.
