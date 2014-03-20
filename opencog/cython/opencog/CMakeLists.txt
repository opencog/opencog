
# Need to use -fno-strict-aliasing when compiling cython code, in order
# to avoid nasty compiler warnings about aliasing.  Cython explicitly
# performs aliasing, in order to emulate python object  inheritance.
# See, for example, https://groups.google.com/forum/#!topic/cython-users/JV1-KvIUeIg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")

INCLUDE_DIRECTORIES(
	${PYTHON_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
)

# Use this as a guide:
# https://github.com/OpenKinect/libfreenect/blob/master/wrappers/python/CMakeLists.txt

IF (HAVE_ATOMSPACE)
	###################### logger ##########################################
	add_custom_command(OUTPUT logger.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o logger.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/logger.pyx"
		DEPENDS "logger.pyx")

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "logger.cpp")

	ADD_LIBRARY(logger_cython SHARED
		logger.cpp
	)

	TARGET_LINK_LIBRARIES(logger_cython
		cogutil
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(logger_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME logger)

	###################### atomspace ####################################
	# How to Cython the .pyx file
	add_custom_command(OUTPUT atomspace.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o atomspace.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/atomspace.pyx"
		DEPENDS "atom.pyx" "classserver.pyx" "atomspace.pyx" "atomspace.pxd"
		        "atomspace_details.pyx" "spacetime.pyx" opencog_atom_types)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "atomspace.cpp")
	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "atomspace_api.h")

	# opencog.atomspace Python bindings
	ADD_LIBRARY(atomspace_cython SHARED
		atomspace.cpp
	)

	ADD_DEPENDENCIES(atomspace_cython opencog_atom_types)

	TARGET_LINK_LIBRARIES(atomspace_cython
		atomspace
		cogutil
		${PYTHON_LIBRARIES}
		${Boost_THREAD_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)

	SET_TARGET_PROPERTIES(atomspace_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME atomspace)

ENDIF (HAVE_ATOMSPACE)

IF (HAVE_ATOMSPACE AND HAVE_GUILE)
	############################## scheme wrapper #####################
	add_custom_command(OUTPUT scheme_wrapper.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -f -o scheme_wrapper.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/scheme_wrapper.pyx"
		DEPENDS "PyScheme.cc" "atomspace.pxd" opencog_atom_types)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "scheme_wrapper.cpp")

	ADD_LIBRARY(scheme_wrapper SHARED
		PyScheme
		scheme_wrapper.cpp
	)

	TARGET_LINK_LIBRARIES(scheme_wrapper
		scheme-shell
		smob
		${PYTHON_LIBRARIES}
	)
	
	SET_TARGET_PROPERTIES(scheme_wrapper PROPERTIES
		PREFIX ""
		OUTPUT_NAME scheme_wrapper)

ENDIF (HAVE_ATOMSPACE AND HAVE_GUILE)


IF (HAVE_SERVER)
	############################ cogserver #########################
	add_custom_command(OUTPUT cogserver.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o cogserver.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/cogserver.pyx"
		DEPENDS "cogserver.pyx" "request.pyx" "agent.pyx" "atomspace.pxd"
		        "cogserver.pxd" opencog_atom_types)

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "cogserver.cpp")

	# opencog.cogserver Python bindings
	ADD_LIBRARY(cogserver_cython SHARED
		cogserver.cpp
	)

	TARGET_LINK_LIBRARIES(cogserver_cython
		server
		${PYTHON_LIBRARIES}
	)

	SET_TARGET_PROPERTIES(cogserver_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME cogserver)

	####################### agent finder ########################
	add_custom_command(OUTPUT agent_finder.cpp agent_finder_api.h
		COMMAND
		${CYTHON_EXECUTABLE} -f -o agent_finder.cpp -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/agent_finder.pyx"
		DEPENDS "agent_finder.pyx" "atomspace.pxd" "cogserver.pxd")

	list(APPEND ADDITIONAL_MAKE_CLEAN_FILES "agent_finder.cpp" "agent_finder_api.h")

	# utility library for PythonModule
	ADD_LIBRARY(agent_finder SHARED
		agent_finder.cpp
	)

	TARGET_LINK_LIBRARIES(agent_finder
		${PYTHON_LIBRARIES}
		${Boost_THREAD_LIBRARY}
		${Boost_SYSTEM_LIBRARY}
	)

	ADD_DEPENDENCIES(agent_finder cogserver_cython atomspace_cython)

	SET_TARGET_PROPERTIES(agent_finder PROPERTIES
		PREFIX ""
		OUTPUT_NAME agent_finder)

ENDIF(HAVE_SERVER)

IF (HAVE_MOSES)
	add_custom_command(OUTPUT pymoses.cpp
		COMMAND
		${CYTHON_EXECUTABLE} -o pymoses.cpp -f -I ${CMAKE_CURRENT_SOURCE_DIR} --cplus "${CMAKE_CURRENT_SOURCE_DIR}/pymoses.pyx"
		DEPENDS "pymoses.pyx" "pymoses.pxd")

	# opencog.pymoses Python bindings
	ADD_LIBRARY(moses_cython SHARED
		pymoses.cpp
	)

	TARGET_LINK_LIBRARIES(moses_cython
		moses_exec
		moses
		cogutil
		comboreduct
		feature_selection
		${PYTHON_LIBRARIES}
		${Boost_PROGRAM_OPTIONS_LIBRARY}
		${Boost_THREAD_LIBRARY}
	)

	IF (HAVE_MPI)
		TARGET_LINK_LIBRARIES (moses_cython
			# These two work for both MPICH2 and OpenMPI, for me.
			${MPI_LIBRARY}
			${MPI_EXTRA_LIBRARY}
		)
	ENDIF (HAVE_MPI)

	SET_TARGET_PROPERTIES(moses_cython PROPERTIES
		PREFIX ""
		OUTPUT_NAME pymoses)

ENDIF (HAVE_MOSES)


IF (WIN32)
	INSTALL (TARGETS agent_finder DESTINATION "${DATADIR}/python")
	INSTALL (TARGETS atomspace_cython cogserver_cython DESTINATION "${DATADIR}/python/opencog")
ELSE (WIN32)
	INSTALL (TARGETS agent_finder LIBRARY DESTINATION "${DATADIR}/python")
	INSTALL (TARGETS atomspace_cython cogserver_cython LIBRARY DESTINATION "${DATADIR}/python/opencog")
ENDIF (WIN32)

INSTALL (FILES 
	opencog/__init__.py
	DESTINATION "${DATADIR}/python/opencog")

