from blending.src.connector.connect_conflict_random import \
    ConnectConflictRandom
from blending.src.connector.connect_conflict_viable import \
    ConnectConflictAllViable
from blending.src.connector.connect_simple import ConnectSimple
from blending.util.blend_config import BlendConfig


__author__ = 'DongMin Kim'


class ConnectorFinder(object):
    def __init__(self, a):
        self.a = a

        self.connectors = {
            ConnectSimple.__name__: ConnectSimple,
            ConnectConflictRandom.__name__: ConnectConflictRandom,
            ConnectConflictAllViable.__name__: ConnectConflictAllViable
        }

    def __str__(self):
        return self.__class__.__name__

    def get_connector(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlendConfig().get_str(self.a, "link-connector")

        connector = self.connectors.get(str(id_or_name))
        if connector is not None:
            return connector(self.a)
        else:
            raise UserWarning('Connector not found.')
