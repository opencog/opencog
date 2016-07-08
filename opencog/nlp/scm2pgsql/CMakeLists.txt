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


ADD_EXECUTABLE (loadscm loadscm.cc)
ADD_EXECUTABLE (scm2pgsql scm2pgsql.cc)
TARGET_LINK_LIBRARIES(loadscm
	scmloader
)
TARGET_LINK_LIBRARIES(scm2pgsql
	scmloader
)

INSTALL (TARGETS scmloader DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")
INSTALL (TARGETS scm2pgsql RUNTIME DESTINATION "bin")
INSTALL (FILES
	SCMLoader.h
	PGStorageDelegate.h
	DESTINATION "include/${PROJECT_NAME}/nlp/scm2pgsql"
)
