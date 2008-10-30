ADD_LIBRARY (gsocket SHARED
	GenericSocket.cc
	GenericModule.cc
)

TARGET_LINK_LIBRARIES(gsocket
)

INSTALL (TARGETS gsocket
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	GenericSocket.h
	GenericModule.h
	DESTINATION "include/opencog/socket"
)
