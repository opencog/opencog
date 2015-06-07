from blender_b.connector.connect_simple import ConnectSimple
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class ConnectorFinder(object):
    def __init__(self, a):
        self.a = a

        self.connector_list = {
            ConnectSimple.__name__: ConnectSimple
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'LINK_CONNECTOR': 'ConnectSimple'
        }
        BlConfig().make_default_config(str(self), default_config)

    def get_connector(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'LINK_CONNECTOR')

        connector = self.connector_list.get(str(id_or_name))
        if connector is not None:
            return connector(self.a)
        else:
            raise UserWarning('Connector not found.')
