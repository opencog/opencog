INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       # NLP atom types
)

ADD_LIBRARY (scmloader SHARED
	SCMLoader
	PGStorageDelegate
)

ADD_DEPENDENCIES (scmloader
	nlp_atom_types            
)

TARGET_LINK_LIBRARIES(scmloader
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS scmloader DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_EXECUTABLE (loadscm loadscm.cc)
ADD_EXECUTABLE (scm2pgsql scm2pgsql.cc)
TARGET_LINK_LIBRARIES(loadscm
	scmloader
)
TARGET_LINK_LIBRARIES(scm2pgsql
	scmloader
)

