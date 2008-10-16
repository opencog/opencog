ADD_LIBRARY (schemes STATIC
	SchemeEval.cc
	SchemeShell.cc
	SchemeSmob.cc
	SchemeSmobAtom.cc
	SchemeSmobNew.cc
	SchemeSmobTV.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/atomspace
	${OPENCOG_SOURCE_DIR}/src/server
	${OPENCOG_SOURCE_DIR}/src/util
)
