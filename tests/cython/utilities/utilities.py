
from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog, finalize_opencog
from opencog.bindlink import execute_atom
import opencog.scheme_wrapper as scheme
from opencog.scheme_wrapper import load_scm, scheme_eval

atomspace = AtomSpace()

def GroundedSchemaNode(node_name):
    return atomspace.add_node(types.GroundedSchemaNode, node_name)

def ConceptNode(node_name):
    return atomspace.add_node(types.ConceptNode, node_name)

def ExecutionOutputLink(schemaNode, listLink):
    return atomspace.add_link(types.ExecutionOutputLink, [schemaNode, listLink])

def ListLink(*args):
    return atomspace.add_link(types.ListLink, args)

# Initialize Scheme
scheme_preload = [  
                    "opencog/atomspace/core_types.scm",
                    "opencog/scm/utilities.scm" 
                 ]
scheme.__init__(atomspace)
for scheme_file in scheme_preload:
    load_scm(atomspace, scheme_file)

initialize_opencog(atomspace, "utilities_test.conf")

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

print "execute: cog-execute"
if (executed):
    print "add_link - executed successfully"
else:
    print "add_link - did NOT execute"

executed = False
execute_atom(   atomspace,
    ExecutionOutputLink( 
        GroundedSchemaNode("py: add_link"),
        ListLink(
            ConceptNode("one"),
            ConceptNode("two") 
        )
    )
)
print "execute: execute_atom"
if (executed):
    print "add_link - executed successfully"
else:
    print "add_link - did NOT execute"


finalize_opencog()
del atomspace
