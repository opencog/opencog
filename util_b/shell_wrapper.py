__author__ = 'DongMin Kim'

import web.api.restapi
from util_b.general_util import *
from util_b.blending_util import *

# Note: Divided to standalone class because I'll remove
class RESTAPILoader:
    def __init__(self, atomspace):
        self.a = atomspace

    def run(self):
        # To avoid debug messages of restapi.
        # import logging

        # logging.basicConfig(level=logging.CRITICAL)
        restapi = web.api.restapi.Start()
        restapi.run("", self.a)


class ShellWrapper:
    def __init__(self):
        self.a = AtomSpace()

        # DEBUG: Run RESTAPI server automatically to see my atomspace.
        rest_api_loader = RESTAPILoader(self.a)
        rest_api_loader.run()

    def ask_user_to_run_or_stop_for_debug(self):
        # BlendingLoggerForDebug().log("Input n to preserve temp links, or delete.")
        # is_delete_temp_link = raw_input()
        # is_preserve_debug_link = 'y'

        # if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
        #     BlendTargetMakerForDebug().backup_debug_link_list()

        # self.experiment_codes_inst.print_atomspace_for_debug()

        BlendingLoggerForDebug().log("Input n to stop, or continue.")
        is_stop = raw_input()

        # if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
        #     BlendTargetMakerForDebug().restore_debug_link_list()

        return is_stop

    def run(self, agent_name):
        # Simulate cogserver environment.
        while 1:
            agent_class = get_class(agent_name)
            agent_inst = agent_class()
            agent_inst.run_by_server = False

            agent_inst.run(self.a)

            is_stop = self.ask_user_to_run_or_stop_for_debug()

            if is_stop == 'n' or is_stop == 'N':
                break



