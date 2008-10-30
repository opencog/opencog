ADD_LIBRARY (smob SHARED
	SchemeEval.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
)

TARGET_LINK_LIBRARIES(smob
	${GUILE_LIBRARIES}
)

INSTALL (TARGETS smob
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	SchemeEval.h
	SchemeSmob.h
	DESTINATION "include/opencog/guile"
)
