LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
	${PROJECT_BINARY_DIR}/opencog/util
	${PROJECT_BINARY_DIR}/opencog/nlp/scm2pgsql
)

LINK_LIBRARIES (
	${ATOMSPACE_LIBRARY}
    scmloader
	server
)

ADD_CXXTEST(SCMLoaderUTest)
