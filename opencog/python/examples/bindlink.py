"""
Example of how to use the pattern matcher BindLink functionality from Python
Based on the following example in the wiki:
http://wiki.opencog.org/w/Pattern_matching#The_Simplified_API
"""

__author__ = 'Cosmo Harrigan'

from opencog.atomspace import AtomSpace, TruthValue, types, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__

atomspace = AtomSpace()
__init__(atomspace)

data = ["opencog/atomspace/core_types.scm",
        "opencog/scm/utilities.scm"]

for item in data:
    load_scm(atomspace, item)

# Define several animals and something of a different type as well
scheme_animals = \
    '''
    (InheritanceLink (ConceptNode "Frog") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "Zebra") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "Deer") (ConceptNode "animal"))
    (InheritanceLink (ConceptNode "Spaceship") (ConceptNode "machine"))
    '''
scheme_eval_h(atomspace, scheme_animals)

# Define a graph search query
scheme_query = \
    '''
    (define find-animals
      (BindLink
        ;; The variable to be bound
        (VariableNode "$var")
        (ImplicationLink
          ;; The pattern to be searched for
          (InheritanceLink
             (VariableNode "$var")
             (ConceptNode "animal")
          )

          ;; The value to be returned.
          (VariableNode "$var")
        )
      )
    )
    '''
scheme_eval_h(atomspace, scheme_query)

# Run the above pattern and print the result
result = scheme_eval_h(atomspace, '(cog-bind find-animals)')
print atomspace[result]
