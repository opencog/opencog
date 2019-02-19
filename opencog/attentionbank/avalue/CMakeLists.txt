ADD_LIBRARY (attentionval
	AttentionValue.cc
	AttentionValueOfLink.cc
)

# Without this, parallel make will race and crap up the generated files.
# ADD_DEPENDENCIES(attentionval opencog_atom_types)

TARGET_LINK_LIBRARIES(attentionval
	value
	${COGUTIL_LIBRARY}
)

INSTALL (TARGETS attentionval EXPORT AtomSpaceTargets
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	AttentionValue.h
	AttentionValueOfLink.h
	DESTINATION "include/opencog/attentionbank/avalue"
)
