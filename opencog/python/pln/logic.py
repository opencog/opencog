__author__ = 'ramin'

from opencog.atomspace import AtomSpace,types
import formulas


class ForwardChainer(object):
    def __init__(self, atomspace = AtomSpace()):
        self.atomspace = atomspace

    def run(self, node):
        result = []
        links = node.incoming

        for link in links:
            if self._is_deductable(link):
                result += self.deduct(node, link)

        return result


    def _is_deductable(self, link):
        allowed_types = [types.InheritanceLink, types.IntensionalInheritanceLink, types.SubsetLink, types.ImplicationLink, types.ExtensionalImplicationLink, types.IntensionalImplicationLink]
        return link.gettype() in allowed_types

    def deduct(self, node, link):
        result = []
        nodes = list(link.out)
        nodes.remove(node)
        for n in nodes:
            out_links = list(n.incoming)
            out_links.remove(link)
            for l in out_links:
                if l.gettype() == link.gettype():
                    a = node
                    b = n
                    c = list(l.out)
                    c.remove(n)
                    c = c[0]
                    ab = link
                    bc = l

                    tv = formulas.deduce(a.tv, b.tv, c.tv, ab.tv, bc.tv)
                    result.append(self.atomspace.add_link(link.type, [a, c], tv))

        return result