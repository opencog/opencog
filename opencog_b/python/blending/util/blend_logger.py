from opencog.atomspace import AtomSpace
from opencog.logger import log
from opencog_b.python.blending.util.blend_config import BlendConfig

__author__ = 'DongMin Kim'

log_config = {
    'log_stdout': None,
    'log_level': None,
    'log_prefix': None,
    'log_postfix': None
}


def log_release_setting():
    global log_config
    log_config = {
        'log_stdout': "False",
        'log_level': "INFO",
        'log_prefix': "[ConceptualBlending]::",
        'log_postfix': ""
    }


def log_debug_setting():
    global log_config
    log_config = {
        'log_stdout': "True",
        'log_level': "WARN",
        'log_prefix': "[CB]::",
        'log_postfix': ""
    }

def debug_log(msg):
    global log_config
    config_backup = log_config.copy()

    log_debug_setting()
    blend_log(msg)

    log_config = config_backup.copy()


# TODO: Remove custom logger or migrate with OpenCog logger
def blend_log(msg):

    if log_config['log_stdout'] == "True":
        log.use_stdout(True)
    else:
        log.use_stdout(False)

    log_prefix = log_config['log_prefix'].replace("\\n", "\n")
    log_postfix = log_config['log_postfix'].replace("\\n", "\n")
    log.log(
        log.string_as_level(log_config['log_level']),
        log_prefix + str(msg) + log_postfix
    )

# Initialize logger.
log_release_setting()
