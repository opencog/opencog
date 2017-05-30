# This is a temporary hack due to the changes made by
# https://github.com/opencog/atomspace/pull/774
from opencog.atomspace import types

def count_to_confidence(count):
    default_k = 800.0 # See TruthValue::DEFAULT_K
    return count / (count + default_k)

def confidence_to_count(conf):
    default_k = 800.0 # See TruthValue::DEFAULT_K
    return default_k * conf / (1.0 - conf)

# This is a temporary hack due to the changes made by
# https://github.com/opencog/opencog/pull/2012 and,
# https://github.com/opencog/atomspace/pull/611
# NOTE: This is similar to scheme `cog-node`.
# FIXME: Should this moved to the atomspace repo and be part
# of opencog.atomspace module?
def get_atoms_by_name(z_type, name, atomspace):
    return filter(lambda x: x.name == name, atomspace.get_atoms_by_type(z_type))
