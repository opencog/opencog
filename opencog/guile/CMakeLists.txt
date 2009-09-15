ADD_LIBRARY (smob SHARED
	SchemeAdHoc.cc
	SchemeEval.cc
	SchemeExec.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobAV.cc
	SchemeSmobGC.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
	SchemeSmobVH.cc
)

TARGET_LINK_LIBRARIES(smob
	${GUILE_LIBRARIES}
	pln
	query
	wsd
)

INSTALL (TARGETS smob
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
	SchemeEval.h
	SchemeSmob.h
	DESTINATION "include/opencog/guile"
)
