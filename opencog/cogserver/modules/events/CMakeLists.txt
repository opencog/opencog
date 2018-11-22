
INCLUDE_DIRECTORIES(${JSONCPP_INCLUDE_DIRS})

ADD_LIBRARY (atomspacepublishermodule SHARED
	AtomSpacePublisherModule
)

TARGET_LINK_LIBRARIES(atomspacepublishermodule
	server
	${ATOMSPACE_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${ZMQ_LIBRARIES}
	tbb
)

INSTALL (TARGETS atomspacepublishermodule
	DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules")
