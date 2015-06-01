
from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog, finalize_opencog
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval

atomspace = AtomSpace()

# Initialize Scheme
scheme_preload = [  
                    "opencog/atomspace/core_types.scm",
                    "opencog/scm/utilities.scm" 
                 ]
scheme.__init__(atomspace)
for scheme_file in scheme_preload:
    load_scm(atomspace, scheme_file)

initialize_opencog(atomspace)

executed = False

def add_link(atom1, atom2):
    global executed
    link = ATOMSPACE.add_link(types.ListLink, [atom1, atom2])
    executed = True
    return link

execute_code = \
    '''
    (cog-execute!
        (ExecutionOutputLink
            (GroundedSchemaNode \"py: add_link\")
            (ListLink
                (ConceptNode \"one\")
                (ConceptNode \"two\")
            )
        )
    )
    '''
scheme_eval(atomspace, execute_code)

if (executed):
    print "add_link - executed successfully"
else:
    print "add_link - did NOT execute"

finalize_opencog()
del atomspace
