import ConfigParser
import os
from opencog.logger import log
import web.api.restapi

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
        (object, ),
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
    return get_class(str(module_path)+"."+str(class_name))


# TODO: link with global config in cogserver
class BlendingConfigLoader(Singleton):
    def __init__(cls):
        super(BlendingConfigLoader, cls).__init__()

        cls.use_config_file = False
        cls.use_blend_target = False
        cls.blending_config = dict()

        config_parser = ConfigParser.ConfigParser()
        config_parser.optionxform = str

        config_parser.read(
            os.path.dirname(os.path.realpath(__file__)) +
            '/../blending.conf'
        )

        sections = config_parser.sections()
        for section in sections:
            option_dict = dict()
            options = config_parser.options(section)
            for option in options:
                option_dict[option] = config_parser.get(section, option)

            cls.blending_config[section] = option_dict

        """
        options = config_parser.options('General')
        for option in options:
            try:
                cls.blending_config[option] = \
                    config_parser.get('General', option)
                if cls.blending_config[option] == -1:
                    BlendingLoggerForDebug().log("skip: %s" % option)
            except:
                BlendingLoggerForDebug().log("exception on %s!" % option)
                cls.blending_config[option] = None
        """

        if cls.get('General', 'USE_CONFIG_FILE') == 'True':
            cls.use_config_file = True
        if cls.get('Blender', 'USE_BLEND_TARGET_FOR_DEBUG') == 'True':
            cls.use_blend_target = True

        cls.is_loaded = True

    def is_use_config_file(cls):
        return cls.use_config_file

    def get(cls, section, key):
        return cls.blending_config[str(section)][str(key)]

    def set(cls, section, key, value):
        cls.blending_config[str(section)][str(key)] = value

    # To get variable using frequently.
    @property
    def is_use(cls):
        """
        :type cls.use_config_file: Boolean
        """
        return cls.use_config_file

    @property
    def is_use_blend_target(cls):
        """
        :type cls.use_blend_target: Boolean
        """
        return cls.use_blend_target


class BlendingLoggerForDebug(Singleton):
    def __init__(cls):
        super(BlendingLoggerForDebug, cls).__init__()

        cls.mode = BlendingConfigLoader().get('General', 'AGENT_MODE')
        if cls.mode == 'Release':
            cls.level = \
                BlendingConfigLoader().get('Log', 'LOG_RELEASE_LEVEL')
            cls.log_prefix = \
                BlendingConfigLoader().get('Log', 'LOG_RELEASE_PREFIX')
            cls.log_postfix = \
                BlendingConfigLoader().get('Log', 'LOG_RELEASE_POSTFIX')
        else:
            log.use_stdout()
            cls.level = \
                BlendingConfigLoader().get('Log', 'LOG_DEBUG_LEVEL')
            cls.log_prefix = \
                BlendingConfigLoader().get('Log', 'LOG_DEBUG_PREFIX')
            cls.log_postfix = \
                BlendingConfigLoader().get('Log', 'LOG_DEBUG_POSTFIX')

    def log(cls, msg):
        cls.log_prefix = cls.log_prefix.replace("\\n", "\n")
        cls.log_postfix = cls.log_postfix.replace("\\n", "\n")
        log.log(
            log.string_as_level(cls.level),
            cls.log_prefix + msg + cls.log_postfix
        )

class RESTAPILoader:
    def __init__(self, a):
        self.a = a

    def run(self, address, port):
        # To avoid debug messages of restapi.
        # import logging
        # logging.basicConfig(level=logging.CRITICAL)
        web.api.restapi.IP_ADDRESS = address
        web.api.restapi.PORT = int(port)
        restapi = web.api.restapi.Start()
        restapi.run("", self.a)
