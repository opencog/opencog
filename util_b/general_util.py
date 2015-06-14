# coding=utf-8
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


# BlAtomConfig: BlendingConfigLoader
# noinspection PyTypeChecker
class BlAtomConfig(Singleton):
    DEFAULT_CONFIG_NAME = "BLEND"
    DEFAULT_CONFIG_SET = \
        {
            "config-format-version",
            "execute-mode",

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
            "decide-sti-max"
        }

    # noinspection PyTypeChecker
    def __init__(cls):
        super(BlAtomConfig, cls).__init__()
        cls.a = None
        cls.is_initialized = False

        # TODO: How to find the scheme module in beautiful?
        scheme_eval(
            cls.a,
            '(add-to-load-path "/home/kdm/atomspace")' +
            '(add-to-load-path "/home/kdm/atomspace/build")' +
            '(add-to-load-path "/home/kdm/atomspace/opencog/scm")' +
            '(add-to-load-path "/home/kdm/opencog")' +
            '(add-to-load-path "/home/kdm/opencog/build")' +
            '(add-to-load-path "/home/kdm/opencog/opencog/scm")' +
            '(add-to-load-path ".")'
        )

        scheme_eval(
            cls.a,
            '(use-modules (opencog))' +
            '(use-modules (opencog query))' +
            '(use-modules (opencog exec))' +
            '(use-modules (opencog rule-engine))'
        )

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
        cls.add(cls.a, "config-format-version", "1")
        cls.add(cls.a, "execute-mode", "Debug")

    def __make_list_link(cls, config_name, config_base, free_var):
        if config_name not in cls.__set:
            raise UserWarning("Wrong config name.")

        return cls.a.add_link(
            types.ListLink,
            [
                cls.a.add_node(types.SchemaNode, cls.name + ':' + config_name),
                config_base,
                free_var
            ]
        )

    def add(cls, a, config_name, config, config_base=None):
        cls.__initialize(a)

        if config_base is None:
            config_base = cls.name

        if (type(config_base) is str) or (type(config_base) is unicode):
            config_base = cls.a.add_node(types.ConceptNode, config_base)

        if (type(config) is str) or (type(config) is unicode):
            config = cls.a.add_node(types.ConceptNode, config)

        free_var = cls.a.add_node(
            types.VariableNode,
            "$" + cls.name + "_free"
        )

        try:
            put_link = cls.a.add_link(
                types.PutLink,
                [
                    cls.__make_list_link(config_name, config_base, free_var),
                    config
                ]
            )
            scheme_eval_h(
                cls.a,
                '(cog-execute! ' +
                '  (cog-atom ' + str(put_link.handle_uuid()) + ')' +
                ')'
            )
        except UserWarning as e:
            BlLogger().log(e)
            BlLogger().log("Skip add config.")
            return

        cls.a.remove(free_var, False)

    # noinspection PyTypeChecker
    def get(cls, a, config_name, config_base=None):
        cls.__initialize(a)

        if config_base is None:
            config_base = cls.name

        if (type(config_base) is str) or (type(config_base) is unicode):
            config_base = cls.a.add_node(types.ConceptNode, config_base)

        free_var = cls.a.add_node(
            types.VariableNode,
            "$" + cls.name + "_free"
        )

        try:
            get_link = cls.a.add_link(
                types.GetLink,
                [
                    cls.a.add_link(types.VariableList, [free_var]),
                    cls.__make_list_link(config_name, config_base, free_var)
                ]
            )
            result_set_uuid = scheme_eval_h(
                cls.a,
                '(cog-execute! ' +
                '  (cog-atom ' + str(get_link.handle_uuid()) + ')' +
                ')'
            )
        except UserWarning as e:
            BlLogger().log(e)
            BlLogger().log("Skip get config.")
            return None

        result_set = cls.a[result_set_uuid]

        try:
            if len(result_set.out) > 1:
                raise UserWarning(
                    "TODO: Currently, config have to keep in unique."
                )
            if len(result_set.out) < 1:
                raise KeyError("Can't find element.")
        except UserWarning as e:
            BlLogger().log(
                str(config_name) + ' in ' +
                str(config_base) + ' = ' +
                str(result_set.out))
            BlLogger().log(e)
            BlLogger().log("Trying to use first element...")

        cls.a.remove(free_var, False)

        return result_set.out[0]

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
        return BlAtomConfig.DEFAULT_CONFIG_SET

    @property
    def name(cls):
        return BlAtomConfig.DEFAULT_CONFIG_NAME


class BlLogger(Singleton):
    def __init__(cls):
        super(BlLogger, cls).__init__()
        cls.log_level = "INFO"
        cls.log_prefix = "[BlendingAgent]::"
        cls.log_postfix = ""

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(cls, a):
        BlAtomConfig().add(a, "log-level", "INFO", "Release")
        BlAtomConfig().add(a, "log-prefix", "[ConceptualBlending]::", "Release")
        BlAtomConfig().add(a, "log-postfix", "", "Release")

        BlAtomConfig().add(a, "log-level", "WARN", "Debug")
        BlAtomConfig().add(a, "log-prefix", "[BA]==>", "Debug")
        BlAtomConfig().add(a, "log-postfix", "", "Debug")

    def change_config(cls, a):
        execute_mode = BlAtomConfig().get_str(a, "execute-mode")
        if execute_mode.upper() == "DEBUG":
            log.use_stdout(True)
        else:
            log.use_stdout(False)

        log_level = BlAtomConfig().get_str(a, "log-level", execute_mode)
        log_prefix = BlAtomConfig().get_str(a, "log-prefix", execute_mode)
        log_postfix = BlAtomConfig().get_str(a, "log-postfix", execute_mode)

        if log_level is not None:
            cls.log_level = log_level
        if log_prefix is not None:
            cls.log_prefix = log_prefix
        if log_postfix is not None:
            cls.log_postfix = log_postfix

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
