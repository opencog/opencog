import opencog.cogserver
from opencog.atomspace import types

# Have a module-global ipshell, which means that variables will remain. It doesn't work yet for some reason.
ipshell = None

# A nice IPython shell that runs within the CogServer.
# NOTE: it uses the CogServer input/output, not the telnet shell

class shell(opencog.cogserver.Request):
    
    # These are printed by the 'help' command
    summary = "OpenCog IPython Shell"
    description = "Usage: pyshell.shell\n\n" \
        "Start an IPython shell in the terminal from which the cogserver "  \
        "was started.  This shell is a python read-evaluate-print loop " \
        "for running arbitrary python commands. The atomspace can be " \
        "accessed as 'space' or 'a'; thus, for example: \n\n" \
        "\th = a.add_node(t.ConceptNode, \"Hello, World!\")\n\tprint h\n\n" \
        "Use ctrl-D to exit the shell."
    is_shell = True

    def run(self,args,atomspace):
        self.run_shell(atomspace)

    def run_shell(self,  space):
        global ipshell
        
        from opencog.atomspace import AtomSpace, types, Atom, Handle, TruthValue
        t = types
        a = space
        namespace = locals().copy()
        doc = """OpenCog Python Shell. Access the main AtomSpace as 'space' or 'a'."""
        try: 
            import sys; sys.argv = ['', '-noconfirm_exit', '-p', 'sh']
            from IPython import embed
            if ipshell == None:
                ipshell = embed(user_ns = namespace)
            ipshell()
        except Exception,  e:
            print e
    #        import code
    #        code.interact(doc, None, namespace)
