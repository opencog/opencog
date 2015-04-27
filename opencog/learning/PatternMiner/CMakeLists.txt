
ADD_LIBRARY (PatternMiner SHARED
	Pattern
	HTree
	PatternMiner
	PatternMinerBF
        PatternMinerDF
)

ADD_DEPENDENCIES(PatternMiner opencog_atom_types)
ADD_DEPENDENCIES(PatternMiner embodiment_atom_types)
ADD_DEPENDENCIES(PatternMiner spacetime_atom_types)

TARGET_LINK_LIBRARIES (PatternMiner
	${COGUTIL_LIBRARY}
)

INSTALL (FILES

	Pattern.h
	HTree.h
	PatternMiner.h

	DESTINATION "include/${PROJECT_NAME}/PatternMiner"
)


