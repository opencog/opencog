ADD_LIBRARY (schemes STATIC
	SchemeShell.cc
	SchemeSmob.cc
)

INCLUDE_DIRECTORIES (
	${OPENCOG_SOURCE_DIR}/src/AtomSpace
	${OPENCOG_SOURCE_DIR}/src/CogServer
	${OPENCOG_SOURCE_DIR}/src/Util
)
