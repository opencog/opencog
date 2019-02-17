ADD_LIBRARY (attentionvalue
	AttentionValue.cc
	AttentionValueOfLink.cc
)

# Without this, parallel make will race and crap up the generated files.
ADD_DEPENDENCIES(attentionvalue opencog_atom_types)

TARGET_LINK_LIBRARIES(attentionvalue
	value
	${COGUTIL_LIBRARY}
)

INSTALL (TARGETS attentionvalue EXPORT AtomSpaceTargets
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AttentionValue.h
	AttentionValueOfLink.h
	DESTINATION "include/opencog/attiontionbank/avalue"
)

# Temporary backwards-compat for opencog.
INSTALL (FILES
	AttentionValue.h
	DESTINATION "include/opencog/atoms/truthvalue"
)

INSTALL (FILES
	AttentionValueOfLink.h
	DESTINATION "include/opencog/atoms/core"
)
