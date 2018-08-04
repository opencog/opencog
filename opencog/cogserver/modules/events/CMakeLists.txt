
ADD_LIBRARY (atomspacepublishermodule SHARED
	AtomSpacePublisherModule
)

TARGET_LINK_LIBRARIES(atomspacepublishermodule
	server
	${ATOMSPACE_LIBRARIES}
	json_spirit
	tbb
)

INSTALL (TARGETS atomspacepublishermodule
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")
