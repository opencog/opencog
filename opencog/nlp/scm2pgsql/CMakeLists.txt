INCLUDE_DIRECTORIES (
	${CMAKE_BINARY_DIR}       
)

ADD_LIBRARY (scmloader SHARED
	SCMLoader
)

ADD_DEPENDENCIES (scmloader
	nlp_atom_types            
)

TARGET_LINK_LIBRARIES(scmloader
	${ATOMSPACE_LIBRARIES}
)

INSTALL (TARGETS scmloader DESTINATION "lib${LIB_DIR_SUFFIX}/opencog")

ADD_EXECUTABLE (loadscm loadscm.cc)
TARGET_LINK_LIBRARIES(loadscm
	scmloader
)

