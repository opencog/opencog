from opencog.logger import log
from opencog_b.python.blending.util.blend_config import BlendConfig

__author__ = 'DongMin Kim'


def __initialize_logger(a):
    if BlendConfig().get(a, "log-initialized") is None:
        BlendConfig().update(a, "log-stdout", "True")

        BlendConfig().update(a, "log-level", "INFO", "Release")
        BlendConfig().update(a, "log-prefix", "[ConceptualBlending]::",
                             "Release")
        BlendConfig().update(a, "log-postfix", "", "Release")

        BlendConfig().update(a, "log-level", "WARN", "Debug")
        BlendConfig().update(a, "log-prefix", "[CB]==>", "Debug")
        BlendConfig().update(a, "log-postfix", "", "Debug")

        BlendConfig().update(a, "log-initialized", "True")


def __get_log_config(a, config_base):
    if a is None:
        return {
            'log_level': "INFO",
            'log_prefix': "[ConceptualBlending]::",
            'log_postfix': ""
        }
    else:
        __initialize_logger(a)

        log_stdout = BlendConfig().get_str(a, "log-stdout", config_base)
        log.use_stdout(True if log_stdout.upper() == "TRUE" else False)

        return {
            'log_level': BlendConfig().get_str(a, "log-level", config_base),
            'log_prefix': BlendConfig().get_str(a, "log-prefix", config_base),
            'log_postfix': BlendConfig().get_str(a, "log-postfix", config_base),
        }


def debug_log(msg, a=None, config_base=None):
    execute_mode = BlendConfig().get_str(a, "execute-mode")
    if execute_mode.upper() == "DEBUG":
        blend_log(msg, config_base)


# TODO: Remove custom logger or migrate with OpenCog logger
def blend_log(msg, a=None, config_base=None):
    log_config_dict = __get_log_config(a, config_base)
    log_prefix = log_config_dict['log_prefix'].replace("\\n", "\n")
    log_postfix = log_config_dict['log_postfix'].replace("\\n", "\n")
    log.log(
        log.string_as_level(log_config_dict['log_level']),
        log_prefix + str(msg) + log_postfix
    )
