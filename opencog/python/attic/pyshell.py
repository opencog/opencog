import opencog.cogserver
from opencog.atomspace import types

# A nice IPython shell that runs within the CogServer.
# NOTE: it uses the CogServer input/output, not the telnet shell

class shell(opencog.cogserver.Request):
    
    # These are printed by the 'help' command
    summary = "OpenCog IPython Shell"
    description = "Usage: pyshell.shell\n\n" \
        "Start an IPython shell in the terminal from which the cogserver "  \
        "was started.  This shell is a python read-evaluate-print loop " \
        "for running arbitrary python commands. The atomspace can be " \
        "accessed as 'atomspace'; thus, for example: \n\n" \
        "\th = atomspace.add_node(types.ConceptNode, \"Hello, World!\")\n\tprint h\n\n" \
        "Use ctrl-D to exit the shell."
    is_shell = True

    def run(self,args,atomspace):
        self.run_shell(atomspace)

    def run_shell(self,  atomspace):
        from opencog.atomspace import types, Atom, Handle, TruthValue
        namespace = locals().copy()
        try: 
            import threading
            import IPython
            from IPython.config.loader import Config

            # Configure the prompts.
            cfg = Config()
            cfg.PromptManager.in_template = "py[\\#]> "
            cfg.PromptManager.out_template = "py[\\#]: "

            # Create an instance of the embeddable shell. 
            from IPython.terminal.embed import InteractiveShellEmbed
            banner = "OpenCog IPython Shell. Access the main AtomSpace as 'atomspace'."
            exit_message = "Leaving Interpreter, back to program."
            ipython_shell = InteractiveShellEmbed(
                    config=cfg,
                    user_ns=namespace, banner1 = banner, 
                    exit_msg = exit_message)                

            # Launch the shell in a new thread so the request doesn't hang
            # the shell that launched this shell.
            shell_thread = threading.Thread(None, ipython_shell, "ipython",())
            shell_thread.start()
        except Exception,  e:
            print e

