
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/opencog/nlp/lojban/CWrapper/)
# The source directory is included becasue there are intermediate haskell
# libraries created that are needed by libopencog-lojban-wrapper.
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/opencog/nlp/lojban/CWrapper/)

LINK_LIBRARIES(
	${ATOMSPACE_LIBRARY}
	server
    LojbanModule
)

ADD_CXXTEST(LojbanModuleTest)
