# This is a temporary hack due to the changes made by
# https://github.com/opencog/atomspace/pull/774
def count_to_confidence(count):
    default_k = 800.0 # See TruthValue::DEFAULT_K
    return count / (count + default_k)
