from examples.python.conceptual_blending.networks.concept_net.concept_net import \
    ConceptNetNetwork
from paul_sally.paul_sally import PaulSallyNetwork
from debate_with_kant.debate_with_kant import DebateWithKantNetwork

__author__ = 'DongMin Kim'


class NetworkLoader:
    def __init__(self, a):
        self.a = a

        self.networks = {
            PaulSallyNetwork.__name__: PaulSallyNetwork,
            DebateWithKantNetwork.__name__: DebateWithKantNetwork,
            ConceptNetNetwork.__name__: ConceptNetNetwork
        }

    def __str__(self):
        return self.__class__.__name__

    def make(self, id_or_name):
        network = self.networks.get(str(id_or_name))
        if network is not None:
            network(self.a).make()
        else:
            raise UserWarning('Test case not found.')

    def make_all(self):
        for network in self.networks.itervalues():
            network(self.a).make()
