# coding=utf-8
import ConfigParser
import os
from opencog.logger import log

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


# BlConfig: BlendingConfigLoader
# TODO: link with global config in cogserver
# 자체 관리가 아닌 cogserver의 config 전역 시스템에 연결하기
class BlConfig(Singleton):
    def __init__(cls):
        super(BlConfig, cls).__init__()

        cls.minimum_config_file_version = 1

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

        if len(sections) > 0:
            for section in sections:
                option_dict = dict()
                options = config_parser.options(section)
                for option in options:
                    option_dict[option] = config_parser.get(section, option)
                cls.blending_config[section] = option_dict
        else:
            cls.use_config_file = False
            log.log(
                log.string_as_level("WARN"),
                "Can't load config file.\n"
                'Please check your config file in here:\n'
                '{0}'
                .format(
                    os.path.dirname(os.path.realpath(__file__)) +
                    '/../blending.conf'
                )
            )
            return

        if cls.get('General', 'USE_CONFIG_FILE') == 'True':
            cls.use_config_file = True

        if int(cls.get('General', 'CONFIG_FILE_VERSION')) < \
                cls.minimum_config_file_version:
            log.log(
                log.string_as_level("WARN"),
                'Please update config file.\n'
                'Required: {0}\n'
                'Current: {1}\n'
                'Now going to use default config.'
                .format(
                    str(cls.minimum_config_file_version),
                    str(cls.get('General', 'CONFIG_FILE_VERSION'))
                )
            )

        if cls.get('ETC', 'USE_BLEND_TARGET_FOR_DEBUG') == 'True':
            cls.use_blend_target = True

        cls.is_loaded = True

    def make_default_config(cls, section, default_config):
        if cls.blending_config.get(str(section)) is not None:
            return
        cls.blending_config[str(section)] = default_config

    def is_use_config_file(cls):
        return cls.use_config_file

    def get_section(cls, section):
        return cls.blending_config.get(str(section))

    def get(cls, section, key):
        section = cls.get_section(section)
        if section is not None:
            return section[str(key)]

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


class BlLogger(Singleton):
    def __init__(cls):
        super(BlLogger, cls).__init__()
        cls.__make_default_config()

        config = BlConfig().get_section(str(cls))

        cls.mode = BlConfig().get('General', 'AGENT_MODE')
        if (cls.mode is None) or (cls.mode == 'Release'):
            cls.level = config.get('LOG_RELEASE_LEVEL')
            cls.log_prefix = config.get('LOG_RELEASE_PREFIX')
            cls.log_postfix = config.get('LOG_RELEASE_POSTFIX')
        else:
            log.use_stdout()
            cls.level = config.get('LOG_DEBUG_LEVEL')
            cls.log_prefix = config.get('LOG_DEBUG_PREFIX')
            cls.log_postfix = config.get('LOG_DEBUG_POSTFIX')

    def __str__(self):
        return self.__class__.__name__

    def __make_default_config(cls):
        default_config = {
            'LOG_RELEASE_LEVEL': 'INFO',
            'LOG_RELEASE_PREFIX': '[BlendingAgent]::',
            'LOG_RELEASE_POSTFIX': ''
        }
        BlConfig().make_default_config(str(cls), default_config)

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
        import web.api.restapi
        # To avoid debug messages of restapi.
        # import logging
        # logging.basicConfig(level=logging.CRITICAL)
        web.api.restapi.IP_ADDRESS = address
        web.api.restapi.PORT = int(port)
        restapi = web.api.restapi.Start()
        restapi.run("", self.a)
