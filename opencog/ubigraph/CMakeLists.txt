ADD_LIBRARY (ubigraph SHARED
    UbigraphModule
    Ubigrapher
    BITUbigrapher
    client
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
    BITUbigrapher.h
    DESTINATION "include/opencog/ubigraph"
)
