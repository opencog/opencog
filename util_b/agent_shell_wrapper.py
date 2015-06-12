__author__ = 'DongMin Kim'

from util_b.general_util import *
from util_b.blending_util import *


class AgentShellWrapper:
    def __init__(self):
        self.a = AtomSpace()

        # Run RESTAPI server automatically to see my atomspace.
        if BlConfig().get('RestAPI', 'USE_RESTAPI') == 'True':
            address = BlConfig().get('RestAPI', 'IP_ADDRESS')
            port = BlConfig().get('RestAPI', 'PORT')
            rest_api_loader = RESTAPILoader(self.a)
            rest_api_loader.run(address, port)

    def ask_user_to_run_or_stop_for_debug(self):
        # BlLogger().log("Input n to preserve temp links, or delete.")
        # is_delete_temp_link = raw_input()
        # is_preserve_debug_link = 'y'

        # if is_preserve_debug_link != 'n' or is_preserve_debug_link != 'N':
        #     BlendTargetMakerForDebug().backup_debug_link_list()

        # self.experiment_codes_inst.print_atomspace_for_debug()

        BlLogger().log("Input n to stop, or continue.")
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


# Start Conceptual Blending.
log.use_stdout()
inst = AgentShellWrapper()
inst.run('blending_agent.BlendingAgent')

# DEBUG: To keep program in running while view my result of coding.
# raw_input("Press enter to exit\n")
