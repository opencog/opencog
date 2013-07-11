SET(CMAKE_C_FLAGS_DEBUG "-O0 -g")

IF (HAVE_PLN)
	SET(BITUBI BITUbigrapher)
ENDIF (HAVE_PLN)


ADD_LIBRARY (ubigraph SHARED
    UbigraphModule
    Ubigrapher
    client
	${BITUBI}
)

INCLUDE_DIRECTORIES (
    ${ODBC_INCLUDE_DIRS}
)

TARGET_LINK_LIBRARIES(ubigraph
    m
    ${CURL_LIBRARIES} 
    ${XMLRPC_LIBRARIES}
)

INSTALL (TARGETS ubigraph
    LIBRARY DESTINATION "lib${LIB_DIR_SUFFIX}/opencog"
)

INSTALL (FILES
    UbigraphAPI.h
    UbigraphModule.h
    Ubigrapher.h
    DESTINATION "include/${PROJECT_NAME}/ubigraph"
)
