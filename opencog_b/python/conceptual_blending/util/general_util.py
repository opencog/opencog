# coding=utf-8
from os.path import expanduser

from opencog.type_constructors import *
from opencog.logger import log
from opencog.scheme_wrapper import *

__author__ = 'DongMin Kim'


class _Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = \
                super(_Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Singleton(
    _Singleton(
        'SingletonMeta',
        (object,),
        {}
    )
):
    pass


def get_class(kls):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__(module)
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


def get_class_by_split_name(module_path, class_name):
    return get_class(str(module_path) + "." + str(class_name))


# http://stackoverflow.com/questions/36932/how-can-i-represent-an-enum-in-python
def enum_simulate(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    reverse = dict((value, key) for key, value in enums.iteritems())
    enums['reverse_mapping'] = reverse
    return type('Enum', (), enums)


# noinspection PyTypeChecker
class PyCogExecute(Singleton):
    def __init__(cls):
        super(PyCogExecute, cls).__init__()
        cls.a = None
        cls.is_initialized = False

    def __initialize(cls, a):
        if cls.a is not a:
            cls.is_initialized = False
            cls.a = a

        if cls.is_initialized is not True:
            cls.is_initialized = True

            # TODO: How to find the scheme module in beautiful?
            scheme_eval(
                cls.a,
                '(add-to-load-path "' + expanduser("~/atomspace") + '")' +
                '(add-to-load-path "' + expanduser("~/atomspace/build") + '")' +
                '(add-to-load-path "' + expanduser("~/atomspace/opencog/scm") + '")' +
                '(add-to-load-path "' + expanduser("~/opencog") + '")' +
                '(add-to-load-path "' + expanduser("~/opencog/build") + '")' +
                '(add-to-load-path "' + expanduser("~/opencog/opencog/scm") + '")' +
                '(add-to-load-path ".")'
            )

            scheme_eval(
                cls.a,
                '(use-modules (opencog))' +
                '(use-modules (opencog query))' +
                '(use-modules (opencog exec))' +
                '(use-modules (opencog rule-engine))' +
                '(load-from-path "utilities.scm")'
            )

    def execute(cls, a, execute_link):
        cls.__initialize(a)

        result_set_uuid = scheme_eval_h(
            cls.a,
            '(cog-execute! ' +
            '  (cog-atom ' + str(execute_link.handle_uuid()) + ')' +
            ')'
        )

        if cls.a.is_valid(result_set_uuid):
            return cls.a[result_set_uuid]
        else:
            return None


# noinspection PyTypeChecker
class BlendConfig(Singleton):
    DEFAULT_CONFIG_NAME = "BLEND"
    DEFAULT_CONFIG_SET = \
        {
            "config-format-version",
            "execute-mode",

            "log-stdout",
            "log-level",
            "log-prefix",
            "log-postfix",

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
            "make-atom-postfix"
        }

    # noinspection PyTypeChecker
    def __init__(cls):
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
        cls.update(cls.a, "config-format-version", "1")
        cls.update(cls.a, "execute-mode", "Release")

    def __blend_config_name_node(cls, config_name):
        if config_name not in cls.__set:
            raise UserWarning("Wrong config name.")

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

        get_link_dict = cls.__create_execute_link("GET", config_dict)
        exist_set = PyCogExecute().execute(a, get_link_dict['execute_link'])
        cls.__clean_up_obsolete_dict(get_link_dict)

        # TODO: Currently, just update one node.
        if len(exist_set.out) > 0:
            config_dict["config"] = exist_set.out[0]
            del_link_dict = cls.__create_execute_link("DEL", config_dict)
            PyCogExecute().execute(a, del_link_dict['execute_link'])
            cls.__clean_up_obsolete_dict(del_link_dict)

        config_dict["config"] = config
        put_link_dict = cls.__create_execute_link("PUT", config_dict)
        PyCogExecute().execute(a, put_link_dict['execute_link'])
        cls.__clean_up_obsolete_dict(put_link_dict)

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

            inh_link_dict = cls.__create_execute_link("INHERITANCE", config_dict)
            parent_set = PyCogExecute().execute(a, inh_link_dict['execute_link'])
            cls.__clean_up_obsolete_dict(inh_link_dict)

            if len(parent_set.out) == 0:
                exist_set = None
                break
            else:
                config_dict["config_base"] = parent_set.out[0]

        try:
            if exist_set is None:
                raise KeyError("Can't find element.")
            if len(exist_set.out) > 1:
                raise UserWarning(
                    "TODO: Currently, config have to keep in unique."
                )

        except UserWarning as e:
            BlLogger().debug_log(
                cls.a,
                str(config_name) + ' in ' +
                str(config_base) + ' = ' +
                str(exist_set.out)
            )
            BlLogger().log(e)
            BlLogger().log("Trying to use first element...")

        ret = exist_set.out[0]
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

    @property
    def __set(cls):
        return BlendConfig.DEFAULT_CONFIG_SET

    @property
    def name(cls):
        return BlendConfig.DEFAULT_CONFIG_NAME


class BlLogger(Singleton):
    def __init__(cls):
        super(BlLogger, cls).__init__()
        cls.log_level = "INFO"
        cls.log_prefix = "[BlendingAgent]::"
        cls.log_postfix = ""

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(cls, a):
        BlendConfig().update(a, "log-stdout", "True")

        BlendConfig().update(a, "log-level", "INFO", "Release")
        BlendConfig().update(a, "log-prefix", "[ConceptualBlending]::", "Release")
        BlendConfig().update(a, "log-postfix", "", "Release")

        BlendConfig().update(a, "log-level", "WARN", "Debug")
        BlendConfig().update(a, "log-prefix", "[BA]==>", "Debug")
        BlendConfig().update(a, "log-postfix", "", "Debug")

    def change_config(cls, a, config_base=None):
        log_stdout = BlendConfig().get_str(a, "log-stdout", config_base)
        log.use_stdout(True if log_stdout.upper() == "TRUE" else False)

        execute_mode = BlendConfig().get_str(
            a, "execute-mode", config_base
        )
        log_level = BlendConfig().get_str(
            a, "log-level", execute_mode
        )
        log_prefix = BlendConfig().get_str(
            a, "log-prefix", execute_mode
        )
        log_postfix = BlendConfig().get_str(
            a, "log-postfix", execute_mode
        )

        if log_level is not None:
            cls.log_level = log_level
        if log_prefix is not None:
            cls.log_prefix = log_prefix
        if log_postfix is not None:
            cls.log_postfix = log_postfix

    def debug_log(cls, a, msg):
        execute_mode = BlendConfig().get_str(a, "execute-mode")
        if execute_mode.upper() == "DEBUG":
            cls.log(msg)

    def log(cls, msg):
        cls.log_prefix = cls.log_prefix.replace("\\n", "\n")
        cls.log_postfix = cls.log_postfix.replace("\\n", "\n")
        log.log(
            log.string_as_level(cls.log_level),
            cls.log_prefix + str(msg) + cls.log_postfix
        )


class RESTAPILoader:
    def __init__(self, a):
        self.a = a

    def run(self, address, port):
        import web.api.restapi
        # To avoid debug messages of restapi.
        # import logging
        # logging.basicConfig(level=logging.CRITICAL)
        web.api.restapi.IP_ADDRESS = address
        web.api.restapi.PORT = int(port)
        restapi = web.api.restapi.Start()
        restapi.run("", self.a)
