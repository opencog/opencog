ADD_SUBDIRECTORY (CWrapper)

# ----------------------------------------
# Lojban cogserver module

LINK_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/CWrapper/)

ADD_LIBRARY (LojbanModule SHARED
    LojbanModule
)

ADD_DEPENDENCIES(LojbanModule LojbanLib)

TARGET_LINK_LIBRARIES (LojbanModule
    opencog-lojban-wrapper-0.1.0.0
    nlp-types
)

INSTALL (TARGETS LojbanModule
	LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog/modules"
)

ADD_EXECUTABLE(parse-lojban Parse)

TARGET_LINK_LIBRARIES(parse-lojban
    opencog-lojban-wrapper-0.1.0.0
    nlp-types
)

INSTALL (TARGETS parse-lojban RUNTIME DESTINATION "bin")
