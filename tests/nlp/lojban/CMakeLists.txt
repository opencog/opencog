
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/opencog/nlp/lojban/CWrapper/)

LINK_LIBRARIES(
	${ATOMSPACE_LIBRARY}
	server
    LojbanModule
)

ADD_CXXTEST(LojbanModuleTest)
