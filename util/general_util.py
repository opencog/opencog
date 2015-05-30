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


class Singleton(_Singleton('SingletonMeta', (object,), {})):
    pass


def get_class(kls):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__(module)
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


class BlendingConfigLoader(Singleton):
    is_loaded = False
    use_config_file = False
    blending_config = dict()

    def __initialize(self):
        if self.is_loaded is True:
            return

        config_parser = ConfigParser.ConfigParser()
        config_parser.read(
            os.path.dirname(os.path.realpath(__file__)) +
            '/../blending.conf'
        )
        options = config_parser.options('General')
        for option in options:
            try:
                self.blending_config[option] = \
                    config_parser.get('General', option)
                if self.blending_config[option] == -1:
                    BlendingLoggerForDebug().log("skip: %s" % option)
            except:
                BlendingLoggerForDebug().log("exception on %s!" % option)
                self.blending_config[option] = None

        self.is_loaded = True

        use_config_file_key = str('BLENDING_USE_CONFIG_FILE').lower()
        if self.blending_config.has_key(use_config_file_key) and \
                self.blending_config[use_config_file_key] == 'True':
            self.use_config_file = True
        else:
            self.use_config_file = False

    def is_use_config_file(self):
        if self.is_loaded is False:
            self.__initialize()

        return self.use_config_file

    def get(self, key):
        if self.is_loaded is False:
            self.__initialize()

        return self.blending_config[str(key).lower()]

    def set(self, key, value):
        if self.is_loaded is False:
            self.__initialize()

        self.blending_config[str(key).lower()] = value


class BlendingLoggerForDebug(Singleton):
    def __init__(self):
        self.mode = BlendingConfigLoader().get('BLENDING_AGENT_MODE')
        if self.mode == 'Release':
            self.level = \
                BlendingConfigLoader().get('BLENDING_LOG_RELEASE_LEVEL')
            self.log_prefix = \
                BlendingConfigLoader().get('BLENDING_LOG_RELEASE_PREFIX')
            self.log_postfix = \
                BlendingConfigLoader().get('BLENDING_LOG_RELEASE_POSTFIX')
        else:
            log.use_stdout()
            self.level = \
                BlendingConfigLoader().get('BLENDING_LOG_DEBUG_LEVEL')
            self.log_prefix = \
                BlendingConfigLoader().get('BLENDING_LOG_DEBUG_PREFIX')
            self.log_postfix = \
                BlendingConfigLoader().get('BLENDING_LOG_DEBUG_POSTFIX')

    def log(self, msg):
        self.log_prefix = self.log_prefix.replace("\\n", "\n")
        self.log_postfix = self.log_postfix.replace("\\n", "\n")
        log.log(
            log.string_as_level(self.level),
            self.log_prefix + msg + self.log_postfix
        )