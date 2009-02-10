INCLUDE_DIRECTORIES (
   ${OPENCOG_SOURCE_DIR}/src/atomspace
   ${OPENCOG_SOURCE_DIR}/src/persist
   ${OPENCOG_SOURCE_DIR}/src/util
)

LINK_DIRECTORIES(
   ${OPENCOG_BINARY_DIR}/src/atomspace
   ${OPENCOG_BINARY_DIR}/src/persist
   ${OPENCOG_BINARY_DIR}/src/util
)

LINK_LIBRARIES(
   atomspace
   persist
   server
   ${Boost_SIGNALS_LIBRARY}
   ${CSOCKETS_LIBRARIES}
   ${OPENSSL_LIBRARIES}
)

ADD_CXXTEST(BasicSaveUTest)

