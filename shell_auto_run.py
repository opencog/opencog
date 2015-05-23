__author__ = 'DongMin Kim'

from shell_blending import ShellBlending

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
inst.__del__()

# DEBUG: To keep program in running while view my result of coding.
# raw_input("Press enter to exit\n")
