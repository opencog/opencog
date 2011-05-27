import opencog.cogserver
from opencog.atomspace import types

# Have a module-global ipshell, which means that variables will remain. It doesn't work yet for some reason.
ipshell = None

# A nice IPython shell that runs within the CogServer.
# NOTE: it uses the CogServer input/output, not the telnet shell
class shell(opencog.cogserver.Request):
    
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
            from IPython.Shell import IPShellEmbed     
            if ipshell == None:
                ipshell = IPShellEmbed(user_ns = namespace)
            ipshell()
        except Exception,  e:
            print e
    #        import code
    #        code.interact(doc, None, namespace)
