from blender_b.connector.connect_simple import ConnectSimple
from util_b.general_util import BlendConfig

__author__ = 'DongMin Kim'


class ConnectorFinder(object):
    def __init__(self, a):
        self.a = a

        self.connectors = {
            ConnectSimple.__name__: ConnectSimple
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
