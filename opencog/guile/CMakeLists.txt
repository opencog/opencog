ADD_LIBRARY (smob SHARED
	SchemeEval.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
	SchemeSocket.cc
	SchemeModule.cc
)

TARGET_LINK_LIBRARIES(smob
	${GUILE_LIBRARIES}
)

INSTALL (TARGETS smob
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	SchemeModule.h
	SchemeEval.h
	SchemeSmob.h
	SchemeSocket.h
	ShemeModule.h
	DESTINATION "include/opencog/guile"
)
