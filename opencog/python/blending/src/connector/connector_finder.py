from blending.src.connector.connect_simple import ConnectSimple
from blending.src.connector.connect_conflict_random import ConnectConflictRandom
from blending.src.connector.connect_conflict_viable import \
    ConnectConflictAllViable
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectorFinder(object):
    """Provide connector instance for user.

    Attributes:
        a: An instance of atomspace.
        last_status: A last status of class.
        connectors: An available link connector list.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type connectors: dict[abc.ABCMeta]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.connectors = {
            ConnectSimple.__name__: ConnectSimple,
            ConnectConflictRandom.__name__: ConnectConflictRandom,
            ConnectConflictAllViable.__name__: ConnectConflictAllViable
        }

    def get_connector(self, config_base):
        self.last_status = blending_status.IN_PROCESS

        connector = self.connectors.get(
            BlendConfig().get_str(self.a, "link-connector", config_base)
        )
        if connector is not None:
            self.last_status = blending_status.SUCCESS
            return connector(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning
