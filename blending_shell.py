from opencog.logger import log
from util_b.shell_wrapper import ShellWrapper

__author__ = 'DongMin Kim'


# Start Conceptual Blending.
log.use_stdout()
inst = ShellWrapper()
inst.run('blending_agent.BlendingAgent')

# DEBUG: To keep program in running while view my result of coding.
# raw_input("Press enter to exit\n")
