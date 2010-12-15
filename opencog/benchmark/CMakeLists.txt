LINK_DIRECTORIES (
	${PROJECT_BINARY_DIR}/opencog/atomspace
)

#INCLUDE_DIRECTORIES (
#)

ADD_EXECUTABLE (AS_no_event_loop
    AtomSpaceBenchmark
    AS_no_event_loop
)

TARGET_LINK_LIBRARIES (AS_no_event_loop
    atomspace
    util
)
