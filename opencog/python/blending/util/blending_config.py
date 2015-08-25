from opencog.logger import log
from opencog.type_constructors import *

from blending.util.blending_util import Singleton
from blending.util.py_cog_execute import PyCogExecute

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker,PyMethodParameters
class BlendConfig(Singleton):
    """Manage the configs of Conceptual Blending.

    It loads & saves config with Nodes in AtomSpace.
    See 'Conceptual Blending Config Format' (blend-config-format.md) document.

    Attributes:
        a: An instance of AtomSpace.
        config_prefix: A prefix name of config nodes.
        config_list: A list of config.
        is_initialized: Displays whether config system was initialized or not.
        execute_link_factory: An instance of ExecuteLinkFactory.
        :type a: AtomSpace
        :type config_prefix_name: str
        :type is_initialized: bool
        :type exe_factory: ExecuteLinkFactory
    """

    def __init__(cls):
        # noinspection PyArgumentList
        super(BlendConfig, cls).__init__()
        cls.a = None
        cls.is_initialized = False
        cls.exe_factory = None
        cls.config_base_node = None
        cls.config_prefix_name = "BLEND"

    def __initialize(cls, a):
        """Initializes a singleton object for specified AtomSpace.

        If the given AtomSpace is different from AtomSpace in class, then
        it starts re-initialize.

        Args:
            a: An instance of AtomSpace to find Atom.
            :param a: AtomSpace
        """
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a
            cls.config_base_node = \
                cls.a.add_node(types.ConceptNode, cls.config_prefix_name)
            cls.exe_factory = \
                ExecuteLinkFactory(cls.a, cls.config_prefix_name)

        if cls.is_initialized is True:
            return

        cls.is_initialized = True
        cls.__make_default_config()

    def __make_default_config(cls):
        """Initialize a default config for this class."""
        # TODO: Possible infinite loop. add->init->make->add->init->...
        cls.update(cls.a, "config-format-version", "20150821")
        cls.update(cls.a, "execute-mode", "Release")

    def __get_config_concept_node(cls, config_base_or_value):
        """Make a ConceptNode with given config base or config value.

        If a config was not given, then it means requesting base ConceptNode
        of config system.
        Or if a given config is string, then it means requesting wrapped
        ConceptNode with given name.
        Else, it just returns given config. In this case given config would be
        ConceptNode.

        Args:
            config_base_or_value: A config to be convert to ConceptNode.
            :param config_base_or_value: str | unicode | Node
        Returns:
            A config base ConceptNode or config value ConceptNode.
            :rtype: ConceptNode
        """
        if config_base_or_value is None:
            return cls.config_base_node
        elif isinstance(config_base_or_value, str):
            return cls.a.add_node(types.ConceptNode, config_base_or_value)
        elif isinstance(config_base_or_value, unicode):
            return cls.a.add_node(types.ConceptNode, config_base_or_value)
        else:
            return config_base_or_value

    def __get_config_name_schema_node(cls, config_name):
        """Make a SchemaNode with given config name.

        If a config was not given, then it means requesting base ConceptNode
        of config system.
        Or if a given config is string, then it means requesting wrapped
        ConceptNode with given name.
        Else, it just returns given config. In this case given config would be
        ConceptNode.

        Args:
            config_base_or_value: A name to be convert to SchemaNode.
            :param config_name: str | unicode | Node
        Returns:
            A config name SchemaNode.
            :rtype: SchemaNode
        Raises:
            ValueError: Config name was not given.
        """
        if config_name is None:
            raise ValueError("Config name was not given.")
        elif isinstance(config_name, str):
            return cls.a.add_node(
                types.SchemaNode,
                cls.config_prefix_name + ':' + config_name
            )
        elif isinstance(config_name, unicode):
            return cls.a.add_node(
                types.SchemaNode,
                cls.config_prefix_name + ':' + config_name
            )
        elif cls.config_prefix_name not in config_name.name:
            return cls.a.add_node(
                types.SchemaNode,
                cls.config_prefix_name + ':' + config_name.name
            )
        else:
            return config_name

    def __get_exist_config_set(cls, config_dict):
        """Find the existing config set.

        Find a value of config that has config_name, starts from config_base.

        Args:
            config_dict: A config dictionary to save information.
            :param config_dict: dict[str, Node]

        Returns:
            exist_set: A found exist atoms set link.
            If a link is None, then means this method couldn't find a proper
            exist config set with given condition.
            :rtype : SetLink
        """
        exist_set = None
        while exist_set is None:
            # Try to find config in current config base.
            exist_set = cls.exe_factory.execute("GET", config_dict)
            if len(exist_set.out) > 0:
                break
            else:
                exist_set = None

            # If config was not found,
            # then try to find in parent config base.
            parent_set = cls.exe_factory.execute("INHERITANCE", config_dict)
            if len(parent_set.out) == 0:
                break

            # If parent config base exists, then change config_base.
            config_dict["config_base"] = parent_set.out[0]

        return exist_set

    def update(cls, a, config_name, config_value, config_base=None):
        """Update config.

        Update value of config that has config_name, starts find from
        config_base. If config_base was not given, then it finds in default
        config_base(Top of config system).

        Args:
            a: An instance of AtomSpace to find Atom.
            config_name: A config to update.
            config_value: A value to assign to config.
            config_base: A base of config system.
            :param a: AtomSpace
            :param config_name: str | unicode | Node
            :param config_value: str | unicode | Node
            :param config_base: str | unicode | Node
        """
        cls.__initialize(a)

        config_base = cls.__get_config_concept_node(config_base)
        config_name = cls.__get_config_name_schema_node(config_name)
        config_value = cls.__get_config_concept_node(config_value)

        config_dict = {
            "config_base": config_base,
            "config_name": config_name,
            "config_value": config_value
        }

        exist_set = cls.__get_exist_config_set(config_dict)

        # If config was found, then delete them first.
        if exist_set is not None:
            config_dict["config_value"] = exist_set.out[0]
            cls.exe_factory.execute("DEL", config_dict)

        # After delete an existing config, then put the new value.
        config_dict["config_value"] = config_value
        cls.exe_factory.execute("PUT", config_dict)

        if exist_set is not None:
            cls.a.remove(exist_set)

    def get(cls, a, config_name, config_base=None):
        """Get config.

        Get value of config that has config_name, starts find from
        config_base. If config_base was not given, then it finds in default
        config_base(Top of config system).

        Args:
            a: An instance of AtomSpace to find Atom.
            config_name: A config to update.
            config_base: A base of config system.
            :param a: AtomSpace
            :param config_name: str | unicode | Node
            :param config_base: str | unicode | Node
        Returns:
            ret: A found config value node.
            :rtype : ConceptNode
        """
        cls.__initialize(a)

        config_base = cls.__get_config_concept_node(config_base)
        config_name = cls.__get_config_name_schema_node(config_name)

        config_dict = {
            "config_name": config_name,
            "config_base": config_base
        }

        exist_set = cls.__get_exist_config_set(config_dict)

        ret = None
        if exist_set is not None:
            if len(exist_set.out) == 1:
                ret = exist_set.out[0]
            elif len(exist_set.out) > 1:
                ret = exist_set.out[0]
                log.warn(
                    "TODO: Currently, config have to keep in unique." +
                    "Trying to use first element..." +
                    str(config_name) + ' in ' +
                    str(config_base) + ' = ' +
                    str(exist_set.out)
                )
            cls.a.remove(exist_set)

        return ret

    def get_str(cls, a, config_name, config_base=None):
        """Get string config.

        Args:
            a: An instance of AtomSpace to find Atom.
            config_name: A config to update.
            config_base: A base of config system.
            :param a: AtomSpace
            :param config_name: str | unicode | Node
            :param config_base: str | unicode | Node

        Returns:
            ret: A found config string value.
            :rtype : str
        """
        ret = cls.get(a, config_name, config_base)
        if ret is not None:
            ret = ret.name
        return ret

    def get_int(cls, a, config_name, config_base=None):
        """Get integer config.

        Args:
            a: An instance of AtomSpace to find Atom.
            config_name: A config to update.
            config_base: A base of config system.
            :param a: AtomSpace
            :param config_name: str | unicode | Node
            :param config_base: str | unicode | Node

        Returns:
            ret: A found config integer value.
            :rtype : int
        """
        ret = cls.get_str(a, config_name, config_base)
        if ret is not None:
            ret = int(ret)
        return ret


class ExecuteLinkFactory:
    """Create the execute links to help blending config manage system.

    Execute links easily with calling execute(call_type, config_dict).
    Or user can make execute links manually with calling
    create(). In this case, user must call the clean_up() method.

    Available call types: PUT, GET, DEL, INHERITANCE
    Structure of config_dict: {
        "config_base": config_base,
        "config_name": config_name,
        "config_value": config_value
    }

    Attributes:
        a: An instance of AtomSpace.
        config_prefix: A prefix name of config nodes.
        :type a: AtomSpace
        :type config_prefix: str
    """

    def __init__(self, a, config_prefix):
        self.a = a
        self.config_prefix = config_prefix

    def __create_put_link(self, config_dict):
        """
        PutLink
            ExecutionLink
                ConceptNode <BLEND:config_name>
                ConceptNode <Config Base>
                VariableNode $X
            <Config>
        Args:
            config_dict: A config dictionary to save information.
            :param config_dict: dict[str, Node]
        Returns:
            A created execute link and obsolete links to remove after use
            execute link.

            ret['execute_link'] = A created put link.
            ret['to_remove_links'] = A obsolete link list.

            :rtype: dict[str, (Link | list[Atom])]
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]
        config_value = config_dict["config_value"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        execution_link = self.a.add_link(
            types.ExecutionLink, [config_name, config_base, free_var]
        )

        put_link = self.a.add_link(
            types.PutLink, [execution_link, config_value]
        )
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, execution_link]
        )

    def __create_get_link(self, config_dict):
        """
        GetLink
            VariableList
                VariableNode $X
            ExecutionLink
                ConceptNode <BLEND:config_name>
                ConceptNode <Config Base>
                VariableNode $X
        Args:
            config_dict: A config dictionary to save information.
            :param config_dict: dict[str, Node]
        Returns:
            A created execute link and obsolete links to remove after use
            execute link.

            ret['execute_link'] = A created get link.
            ret['to_remove_links'] = A obsolete link list.

            :rtype: dict[str, (Link | list[Atom])]
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        execution_link = self.a.add_link(
            types.ExecutionLink, [config_name, config_base, free_var]
        )

        get_link = self.a.add_link(
            types.GetLink, [free_var_list, execution_link]
        )
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, execution_link, get_link]
        )

    def __create_del_link(self, config_dict):
        """
        PutLink
            DeleteLink
                ExecutionLink
                    ConceptNode <BLEND:config_name>
                    ConceptNode <Config Base>
                    VariableNode $X
            <Config>
        Args:
            config_dict: A config dictionary to save information.
            :param config_dict: dict[str, Node]
        Returns:
            A created execute link and obsolete links to remove after use
            execute link.

            ret['execute_link'] = A created put link.
            ret['to_remove_links'] = A obsolete link list.

            :rtype: dict[str, (Link | list[Atom])]
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]
        config_value = config_dict["config_value"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        execution_link = self.a.add_link(
            types.ExecutionLink, [config_name, config_base, free_var]
        )

        del_link = self.a.add_link(types.DeleteLink, [execution_link])
        put_link = self.a.add_link(types.PutLink, [del_link, config_value])
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, execution_link, del_link]
        )

    def __create_inheritance_link(self, config_dict):
        """
        GetLink
            VariableList
                VariableNode $X
            InheritanceLink
                ConceptNode <Config Base>
                VariableNode $X
        Args:
            config_dict: A config dictionary to save information.
            :param config_dict: dict[str, Node]
        Returns:
            A created execute link and obsolete links to remove after use
            execute link.

            ret['execute_link'] = A created get link.
            ret['to_remove_links'] = A obsolete link list.

            :rtype: dict[str, (Link | list[Atom])]
        """
        config_base = config_dict["config_base"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        execution_link = self.a.add_link(
            types.InheritanceLink, [config_base, free_var]
        )

        get_link = self.a.add_link(
            types.GetLink, [free_var_list, execution_link]
        )
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, execution_link]
        )

    def create(self, call_type, config_dict):
        """Factory method to create an execute link.

        Available call types: PUT, GET, DEL, INHERITANCE
        Structure of config_dict: {
            "config_base": config_base,
            "config_name": config_name,
            "config_value": config_value
        }

        Args:
            call_type: Type of creation.
            config_dict: A config dictionary to save information.
            :param call_type: str
            :param config_dict: dict[str, Node]
        Returns:
            A created execute link and obsolete links to remove after use
            execute link.

            ret['execute_link'] = A created execute link.
            ret['to_remove_links'] = A obsolete link list.

            :rtype: dict[str, (Link | list[Atom])]
        Raises:
            UserWarning: Requested an invalid call type.
        """
        if call_type == "PUT":
            return self.__create_put_link(config_dict)
        elif call_type == "GET":
            return self.__create_get_link(config_dict)
        elif call_type == "DEL":
            return self.__create_del_link(config_dict)
        elif call_type == "INHERITANCE":
            return self.__create_inheritance_link(config_dict)
        else:
            raise UserWarning("Non valid API!")

    def execute(self, call_type, config_dict):
        """Automatically create an execute link, execute that link, and
        clean up the obsolete links.

        Available call types: PUT, GET, DEL, INHERITANCE
        Structure of config_dict: {
            "config_base": config_base,
            "config_name": config_name,
            "config_value": config_value
        }

        Args:
            call_type: Type of execution.
            config_dict: A config dictionary to save information.
            :param call_type: str
            :param config_dict: dict[str, Node]
        Returns:
            A result of executing the link.
            :rtype: Link
        """
        link_dict = self.create(call_type, config_dict)
        result = PyCogExecute().execute(self.a, link_dict['execute_link'])
        self.clean_up(link_dict)
        return result

    def clean_up(self, obsolete_dict_or_list):
        """Clean up the obsolete links.

        Find the atoms in given obsolete_dict and remove theirs from AtomSpace.

        Args:
            obsolete_dict: A dictionary has obsolete links.
            :param obsolete_dict_or_list: (dict[str, Link] | list[Link])
        """
        if isinstance(obsolete_dict_or_list, dict):
            obsolete_links = obsolete_dict_or_list.values()
        elif isinstance(obsolete_dict_or_list, list):
            obsolete_links = obsolete_dict_or_list
        else:
            obsolete_links = [obsolete_dict_or_list]
        for link in obsolete_links:
            if isinstance(link, list):
                obsolete_links.extend(link)
                obsolete_links.remove(link)
            else:
                self.a.remove(link)
                obsolete_links.remove(link)
                del link
        del obsolete_dict_or_list
