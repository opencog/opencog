from os.path import expanduser
from examples.python.conceptual_blending.networks.base_network import \
    BaseNetwork
from opencog.scheme_wrapper import scheme_eval

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker
class ConceptNetNetwork(BaseNetwork):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        # TODO: How to find the scheme module in beautiful?
        usr = expanduser("~/")

        scheme_eval(
            self.a,
            '(add-to-load-path "' + usr + "atomspace" + '")' +
            '(add-to-load-path "' + usr + "atomspace/build" + '")' +
            '(add-to-load-path "' + usr + "atomspace/build/opencog" + '")' +
            '(add-to-load-path "' + usr + "atomspace/opencog" + '")' +
            '(add-to-load-path "' + usr + "atomspace/opencog/scm" + '")' +
            '(add-to-load-path "' + usr + "opencog" + '")' +
            '(add-to-load-path "' + usr + "opencog/build" + '")' +
            '(add-to-load-path "' + usr + "opencog/build/opencog" + '")' +
            '(add-to-load-path "' + usr + "opencog/opencog" + '")' +
            '(add-to-load-path "' + usr + "opencog/opencog/scm" + '")' +
            '(add-to-load-path ".")'
        )

        scheme_eval(
            self.a,
            '(use-modules (opencog))' +
            '(use-modules (opencog query))' +
            '(use-modules (opencog exec))' +
            '(use-modules (opencog rule-engine))' +
            '(load-from-path "utilities.scm")'
        )

    def __str__(self):
        return self.__class__.__name__

    def make(self):
        usr = expanduser("~/")
        scheme_eval(
            self.a,
            '(load-from-path "' + usr + "opencog/build/opencog/spacetime/spacetime_types.scm" + '")' +
            '(load-from-path "' + usr + "test-datasets/conceptnet/conceptnet4.scm" + '")'
        )
