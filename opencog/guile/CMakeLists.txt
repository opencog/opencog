ADD_LIBRARY (scheme SHARED
	SchemeShell.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
    SchemeSocket.cc
    SchemeModule.cc
)

TARGET_LINK_LIBRARIES(scheme
	${GUILE_LIBRARIES}
)

INSTALL (TARGETS scheme
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	SchemeModule.h
	SchemeShell.h
	SchemeSmob.h
	SchemeSocket.h
	ShemeModule.h
	DESTINATION "include/opencog/guile"
)
