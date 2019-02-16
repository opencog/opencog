ADD_LIBRARY (attentionvalue
	AttentionValue.cc
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
	DESTINATION "include/opencog/atoms/truthvalue"
)
