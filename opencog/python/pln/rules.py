__author__ = 'ramin'

from opencog.atomspace import types
import formulas


class Rule(object):
    def __init__(self, atomspace=None):
        self.atomspace = atomspace

    def can_use(self, link):
        pass

    def run(self, node, link):
        pass


class DeductionRule(Rule):
    def can_use(self, link):
        allowed_types = [types.InheritanceLink]#, types.IntensionalInheritanceLink, types.SubsetLink, types.ImplicationLink, types.ExtensionalImplicationLink, types.IntensionalImplicationLink]
        return link.type in allowed_types

    def run(self, node, link):
        result = []
        nodes = list(link.out)
        nodes.remove(node)
        for n in nodes:
            out_links = list(n.incoming)
            out_links.remove(link)
            for l in out_links:
                if l.type == link.type:
                    a = node
                    b = n
                    c = list(l.out)
                    c.remove(n)
                    c = c[0]
                    ab = link
                    bc = l

                    tv = formulas.deduction(a.tv, b.tv, c.tv, ab.tv, bc.tv)
                    # TODO should check if the link exists or not, revise the tv accordingly
                    result.append(self.atomspace.add_link(link.type, [a, c], tv))

        return result
