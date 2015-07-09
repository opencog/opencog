from opencog.logger import log
from opencog.type_constructors import *

from blending.util.blending_util import Singleton
from blending.util.py_cog_execute import PyCogExecute

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker,PyMethodParameters
class BlendConfig(Singleton):
    """Manages configs of Conceptual Blending.

    It loads & saves config with Nodes in AtomSpace.
    See 'Conceptual Blending Config Format' (blend-config-format.md) document.

    Attributes:
        a: An instance of atomspace.
        config_prefix: A prefix name of config nodes.
        config_list: A list of config.
        is_initialized: Displays whether config system was initialized or not.
        execute_link_factory: An instance of ExecuteLinkFactory.
        :type a: opencog.atomspace.AtomSpace
        :type config_prefix: str
        :type config_list: dict
        :type is_initialized: bool
        :type execute_link_factory: ExecuteLinkFactory
    """

    config_prefix = "BLEND"
    config_list = {
        "execute-mode",

        "atoms-chooser",
        "blending-decider",
        "new-blend-atom-maker",
        "link-connector",

        "choose-atom-type",
        "choose-least-count",
        "choose-sti-min",
        "choose-sti-max",

        "decide-result-atoms-count",
        "decide-sti-min",
        "decide-sti-max",

        "make-atom-prefix",
        "make-atom-separator",
        "make-atom-postfix",

        "connect-check-type",
        "connect-strength-diff-limit",
        "connect-confidence-above-limit",
        "connect-viable-atoms-count-limit"
    }

    # TODO: Private setting handling
    config_list.update({
        "config-format-version"
    })

    def __init__(cls):
        # noinspection PyArgumentList
        super(BlendConfig, cls).__init__()
        cls.a = None
        cls.is_initialized = False
        cls.execute_link_factory = None

    def __initialize(cls, a):
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a
            cls.execute_link_factory = \
                ExecuteLinkFactory(cls.a, cls.config_prefix)

        if cls.is_initialized is True:
            return

        cls.is_initialized = True
        cls.__make_default_config()

    def __make_default_config(cls):
        # blend
        cls.a.add_node(types.ConceptNode, cls.name)

        # blend config
        # TODO: Possible infinite loop. add->init->make->add->init->...
        cls.update(cls.a, "config-format-version", "2")
        cls.update(cls.a, "execute-mode", "Release")

    def __get_config_concept_node(cls, config):
        if config is None:
            return cls.a.add_node(types.ConceptNode, cls.name)
        elif isinstance(config, str) or isinstance(config, unicode):
            return cls.a.add_node(types.ConceptNode, config)
        elif config is type(dict):
            raise DeprecationWarning("Passing config dict was deprecated.")
        else:
            return config

    def __execute_wrap(cls, call_type, config_dict):
        link_dict = cls.execute_link_factory.create(call_type, config_dict)
        result = PyCogExecute().execute(cls.a, link_dict['execute_link'])
        cls.execute_link_factory.clean_up(link_dict)
        return result

    def update(cls, a, config_name, config, config_base=None):
        cls.__initialize(a)

        config = cls.__get_config_concept_node(config)
        config_base = cls.__get_config_concept_node(config_base)

        config_dict = {
            "config_name": config_name,
            "config_base": config_base,
            "config": config
        }

        exist_set = None
        while True:
            exist_set = cls.__execute_wrap("GET", config_dict)
            if len(exist_set.out) > 0:
                break

            parent_set = cls.__execute_wrap("INHERITANCE", config_dict)
            if len(parent_set.out) == 0:
                exist_set = None
                break
            else:
                config_dict["config_base"] = parent_set.out[0]

        if exist_set is not None:
            config_dict["config"] = exist_set.out[0]
            cls.__execute_wrap("DEL", config_dict)

        config_dict["config"] = config
        cls.__execute_wrap("PUT", config_dict)

        if exist_set is not None:
            cls.a.remove(exist_set)

    def get(cls, a, config_name, config_base=None):
        cls.__initialize(a)

        config_base = cls.__get_config_concept_node(config_base)

        config_dict = {
            "config_name": config_name,
            "config_base": config_base
        }

        exist_set = None
        while True:
            exist_set = cls.__execute_wrap("GET", config_dict)
            if len(exist_set.out) > 0:
                break

            parent_set = cls.__execute_wrap("INHERITANCE", config_dict)
            if len(parent_set.out) == 0:
                exist_set = None
                break
            else:
                config_dict["config_base"] = parent_set.out[0]

        if exist_set is not None:
            ret = None
            if len(exist_set.out) < 1:
                ret = None
            elif len(exist_set.out) == 1:
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
        ret = cls.get(a, config_name, config_base)
        if ret is not None:
            ret = ret.name
        return ret

    def get_int(cls, a, config_name, config_base=None):
        ret = cls.get_str(a, config_name, config_base)
        if ret is not None:
            ret = int(ret)
        return ret

    # noinspection PyPropertyDefinition
    @property
    def name(cls):
        return BlendConfig.config_prefix


class ExecuteLinkFactory:
    """Create execute links to help config manage system.

    Make execute links easily with calling create(call_type, config_dict).

    Available call types: PUT, GET, DEL, INHERITANCE
    Structure of config_dict: {
        "config_name": config_name,
        "config_base": config_base,
        "config": config
    }

    Attributes:
        a: An instance of atomspace.
        config_prefix: A prefix name of config nodes.
        :type a: opencog.atomspace.AtomSpace
        :type config_prefix: str
    """

    def __init__(self, a, config_prefix):
        self.a = a
        self.config_prefix = config_prefix

    def __get_prefixed_config_name(self, config_name):
        return self.a.add_node(
            types.SchemaNode,
            self.config_prefix + ':' + config_name
        )

    def __create_put_link(self, config_dict):
        """
        PutLink
            ListLink
                ConceptNode <BLEND:config_name>
                ConceptNode <Config Base>
                VariableNode $X
            <Config>
        Args:
            :param config_dict: dict
        Returns:
            :rtype: dict
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]
        config = config_dict["config"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        list_link = self.a.add_link(
            types.ListLink,
            [
                self.__get_prefixed_config_name(config_name),
                config_base,
                free_var
            ]
        )

        put_link = self.a.add_link(types.PutLink, [list_link, config])
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, list_link]
        )

    def __create_get_link(self, config_dict):
        """
        GetLink
            VariableList
                VariableNode $X
            ListLink
                ConceptNode <BLEND:config_name>
                ConceptNode <Config Base>
                VariableNode $X
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        list_link = self.a.add_link(
            types.ListLink,
            [
                self.__get_prefixed_config_name(config_name),
                config_base,
                free_var
            ]
        )

        get_link = self.a.add_link(types.GetLink, [free_var_list, list_link])
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, list_link, get_link]
        )

    def __create_del_link(self, config_dict):
        """
        PutLink
            DeleteLink
                ListLink
                    ConceptNode <BLEND:config_name>
                    ConceptNode <Config Base>
                    VariableNode $X
            <Config>
        """
        config_name = config_dict["config_name"]
        config_base = config_dict["config_base"]
        config = config_dict["config"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        list_link = self.a.add_link(
            types.ListLink,
            [
                self.__get_prefixed_config_name(config_name),
                config_base,
                free_var
            ]
        )

        del_link = self.a.add_link(types.DeleteLink, [list_link])
        put_link = self.a.add_link(types.PutLink, [del_link, config])
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, list_link, del_link]
        )

    def __create_inheritance_link(self, config_dict):
        """
        GetLink
            VariableList
                VariableNode $X
            InheritanceLink
                ConceptNode <Config Base>
                VariableNode $X
        """
        config_base = config_dict["config_base"]

        free_var = self.a.add_node(
            types.VariableNode, "$" + self.config_prefix + "_free"
        )
        free_var_list = self.a.add_link(types.VariableList, [free_var])

        list_link = self.a.add_link(
            types.InheritanceLink, [config_base, free_var]
        )

        get_link = self.a.add_link(types.GetLink, [free_var_list, list_link])
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, list_link]
        )

    def create(self, call_type, config_dict):
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

    def clean_up(self, obsolete_dict):
        obsolete_links = obsolete_dict.values()
        for link in obsolete_links:
            if isinstance(link, list):
                obsolete_links.extend(link)
                obsolete_links.remove(link)
            else:
                self.a.remove(link)
