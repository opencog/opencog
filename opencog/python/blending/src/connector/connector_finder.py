from blending.src.connector.connect_simple import ConnectSimple
from blending.src.connector.connect_conflict_random import ConnectConflictRandom
from blending.src.connector.connect_conflict_viable import \
    ConnectConflictAllViable
from blending.src.connector.connect_conflict_interaction_information import \
    ConnectConflictInteractionInformation
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConnectorFinder(object):
    """Provider class to make link connector instance.

    This provider will made the instance of link connector, and returns them to
    the blender.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        connectors: An available link connector list.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type connectors: dict[BaseConnector]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.connectors = {
            ConnectSimple.__name__: ConnectSimple,
            ConnectConflictRandom.__name__: ConnectConflictRandom,
            ConnectConflictAllViable.__name__: ConnectConflictAllViable,
            ConnectConflictInteractionInformation.__name__:
                ConnectConflictInteractionInformation
        }

    def get_connector(self, config_base):
        """Provider method for link connector.

        Args:
            config_base: A Node to save custom config.
            :param config_base: Atom
        Returns:
            The instance of link connector.
            :rtype : BaseChooser
        Raises:
            UserWarning: Can't find the link connector with given name.
        """
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
