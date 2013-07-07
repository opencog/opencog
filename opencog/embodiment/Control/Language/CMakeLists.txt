ADD_LIBRARY (Language SHARED
    LanguageComprehension
    FramesToRelexRuleEngine
    NLGenClient
    OutputRelex
)

ADD_DEPENDENCIES(Language opencog_atom_types)
ADD_DEPENDENCIES(Language spacetime_types)
ADD_DEPENDENCIES(Language nlp_atom_types)
ADD_DEPENDENCIES(Language embodiment_atom_types)

TARGET_LINK_LIBRARIES (Language
	atomspace
)

