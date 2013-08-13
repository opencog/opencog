__author__ = 'ramin'

from rules import Rule


class ForwardChainer(object):
    def __init__(self, atomspace=None):
        self.atomspace = atomspace
        self.rules = []

    def run(self, node):
        result = []
        links = node.incoming
        for rule in self.rules:
            for link in links:
                if rule.can_use(link):
                    result += rule.run(node, link)
        return result

    def add_rule(self, rule):
        assert isinstance(rule, Rule)
        assert type(rule) != Rule

        self.rules.append(rule)