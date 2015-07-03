from opencog.type_constructors import *
from blending.util.blend_logger import debug_log, fine_log
from blending.util.general_util import Singleton
from blending.util.py_cog_execute import PyCogExecute

__author__ = 'DongMin Kim'


# noinspection PyTypeChecker
class BlendConfig(Singleton):
    DEFAULT_CONFIG_NAME = "BLEND"
    DEFAULT_CONFIG_SET = {
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
    DEFAULT_CONFIG_SET.update({
        "config-format-version"
    })

    # noinspection PyTypeChecker
    def __init__(cls):
        # noinspection PyArgumentList
        super(BlendConfig, cls).__init__()
        cls.a = None
        cls.is_initialized = False

    def __initialize(cls, a):
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a

        if cls.is_initialized is not True:
            cls.is_initialized = True
            cls.__make_default_config()

    def __make_default_config(cls):
        # blend
        cls.a.add_node(types.ConceptNode, cls.name)

        # blend config
        # TODO: Inherit chooser, decider, ... to BLEND?
        # TODO: Possible infinite loop. add->init->make->add->init->...
        cls.update(cls.a, "config-format-version", "2")
        cls.update(cls.a, "execute-mode", "Release")

    def __blend_config_name_node(cls, config_name):
        if config_name not in cls.DEFAULT_CONFIG_SET:
            raise UserWarning("Wrong config name: " + str(config_name))

        return cls.a.add_node(types.SchemaNode, cls.name + ':' + config_name)

    def __create_put_link(cls, config_dict):
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

        free_var = cls.a.add_node(types.VariableNode,
                                  "$" + cls.name + "_free")
        free_var_list = cls.a.add_link(types.VariableList, [free_var])
        list_link = cls.a.add_link(
            types.ListLink,
            [cls.__blend_config_name_node(config_name), config_base,
             free_var]
        )
        put_link = cls.a.add_link(types.PutLink, [list_link, config])
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, list_link]
        )

    def __create_get_link(cls, config_dict):
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

        free_var = cls.a.add_node(types.VariableNode,
                                  "$" + cls.name + "_free")
        free_var_list = cls.a.add_link(types.VariableList, [free_var])
        list_link = cls.a.add_link(
            types.ListLink,
            [cls.__blend_config_name_node(config_name), config_base,
             free_var]
        )
        get_link = cls.a.add_link(types.GetLink, [free_var_list, list_link])
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, list_link, get_link]
        )

    def __create_del_link(cls, config_dict):
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

        free_var = cls.a.add_node(types.VariableNode,
                                  "$" + cls.name + "_free")
        free_var_list = cls.a.add_link(types.VariableList, [free_var])
        list_link = cls.a.add_link(
            types.ListLink,
            [cls.__blend_config_name_node(config_name), config_base,
             free_var]
        )
        del_link = cls.a.add_link(types.DeleteLink, [list_link])
        put_link = cls.a.add_link(types.PutLink, [del_link, config])
        return dict(
            execute_link=put_link,
            to_remove_links=[free_var, free_var_list, list_link, del_link]
        )

    def __create_inheritance_link(cls, config_dict):
        """
        GetLink
            VariableList
                VariableNode $X
            InheritanceLink
                ConceptNode <Config Base>
                VariableNode $X
        """
        config_base = config_dict["config_base"]

        free_var = cls.a.add_node(types.VariableNode,
                                  "$" + cls.name + "_free")
        free_var_list = cls.a.add_link(types.VariableList, [free_var])
        list_link = cls.a.add_link(
            types.InheritanceLink, [config_base, free_var]
        )
        get_link = cls.a.add_link(types.GetLink, [free_var_list, list_link])
        return dict(
            execute_link=get_link,
            to_remove_links=[free_var, free_var_list, list_link]
        )

    def __create_execute_link(cls, call_type, config_dict):
        if call_type == "PUT":
            return cls.__create_put_link(config_dict)
        elif call_type == "GET":
            return cls.__create_get_link(config_dict)
        elif call_type == "DEL":
            return cls.__create_del_link(config_dict)
        elif call_type == "INHERITANCE":
            return cls.__create_inheritance_link(config_dict)
        else:
            raise AttributeError("Non valid API!")

    def __wrap_config_name(cls, config):
        if config is None:
            return cls.a.add_node(types.ConceptNode, cls.name)
        elif isinstance(config, str) or isinstance(config, unicode):
            return cls.a.add_node(types.ConceptNode, config)
        else:
            return config

    def __clean_up_obsolete_dict(cls, obsolete_dict):
        obsolete_links = obsolete_dict.values()
        for link in obsolete_links:
            if isinstance(link, list):
                obsolete_links.extend(link)
                obsolete_links.remove(link)
            else:
                cls.a.remove(link)

    def update(cls, a, config_name, config, config_base=None):
        cls.__initialize(a)

        config = cls.__wrap_config_name(config)
        config_base = cls.__wrap_config_name(config_base)

        config_dict = {
            "config_name": config_name,
            "config_base": config_base,
            "config": config
        }

        exist_set = None
        while True:
            get_link_dict = cls.__create_execute_link("GET", config_dict)
            exist_set = PyCogExecute().execute(a, get_link_dict['execute_link'])
            cls.__clean_up_obsolete_dict(get_link_dict)

            if len(exist_set.out) > 0:
                break

            inh_link_dict = cls.__create_execute_link(
                "INHERITANCE",
                config_dict
            )
            parent_set = PyCogExecute().execute(
                a, inh_link_dict['execute_link']
            )
            cls.__clean_up_obsolete_dict(inh_link_dict)

            if len(parent_set.out) == 0:
                exist_set = None
                break
            else:
                config_dict["config_base"] = parent_set.out[0]

        if exist_set is not None:
            config_dict["config"] = exist_set.out[0]
            del_link_dict = cls.__create_execute_link("DEL", config_dict)
            PyCogExecute().execute(a, del_link_dict['execute_link'])
            cls.__clean_up_obsolete_dict(del_link_dict)

        config_dict["config"] = config
        put_link_dict = cls.__create_execute_link("PUT", config_dict)
        PyCogExecute().execute(a, put_link_dict['execute_link'])
        cls.__clean_up_obsolete_dict(put_link_dict)

        if exist_set is not None:
            cls.a.remove(exist_set)

    # noinspection PyTypeChecker
    def get(cls, a, config_name, config_base=None):
        cls.__initialize(a)

        config_base = cls.__wrap_config_name(config_base)

        config_dict = {
            "config_name": config_name,
            "config_base": config_base
        }

        exist_set = None
        while True:
            get_link_dict = cls.__create_execute_link("GET", config_dict)
            exist_set = PyCogExecute().execute(a, get_link_dict['execute_link'])
            cls.__clean_up_obsolete_dict(get_link_dict)

            if len(exist_set.out) > 0:
                break

            inh_link_dict = cls.__create_execute_link(
                "INHERITANCE", config_dict
            )
            parent_set = PyCogExecute().execute(
                a, inh_link_dict['execute_link']
            )
            cls.__clean_up_obsolete_dict(inh_link_dict)

            if len(parent_set.out) == 0:
                exist_set = None
                break
            else:
                config_dict["config_base"] = parent_set.out[0]

        if exist_set is None:
            return None

        ret = None
        try:
            if len(exist_set.out) < 1:
                ret = None
            elif len(exist_set.out) == 1:
                ret = exist_set.out[0]
            elif len(exist_set.out) > 1:
                ret = exist_set.out[0]
                raise UserWarning(
                    "TODO: Currently, config have to keep in unique." +
                    "Trying to use first element..." +
                    str(config_name) + ' in ' +
                    str(config_base) + ' = ' +
                    str(exist_set.out)
                )
        except UserWarning as e:
            debug_log(e)

        cls.a.remove(exist_set)
        fine_log(ret)
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

    @property
    def name(cls):
        return BlendConfig.DEFAULT_CONFIG_NAME
