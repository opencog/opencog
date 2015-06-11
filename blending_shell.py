from opencog.logger import log
from util_b.agent_shell_wrapper import AgentShellWrapper

__author__ = 'DongMin Kim'


# Start Conceptual Blending.
log.use_stdout()
inst = AgentShellWrapper()
inst.run('blending_agent.BlendingAgent')

# DEBUG: To keep program in running while view my result of coding.
# raw_input("Press enter to exit\n")
