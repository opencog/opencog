
INCLUDE_DIRECTORIES(
	${HYPERTABLE_INCLUDE_DIRS}
#    /home/jeremy/hypertable-0.9.3/0.9.2.4/include
)

ADD_LIBRARY(hypertabledb STATIC
	AtomspaceHTabler.cc
)

ADD_EXECUTABLE(hypertable-test test.cc)

# This is a hack needed only if ${HYPERTABLE_LIBRARIES}
# got set up bad ... see below ... 
# LINK_DIRECTORIES(
#     /usr/local/hypertable/0.9.2.4/lib
# )

TARGET_LINK_LIBRARIES(hypertable-test
    atomspace
    server

    ${HYPERTABLE_LIBRARIES}
    hypertabledb

#   I don't think we need to list all of these here .. 
#   I think that ${HYPERTABLE_LIBRARIES} will take care of things,
#   right?  If not, then edit FindHyperTable.cmake, as needed ... 
#
#    HyperComm
#    HyperDfsBroker
#    HyperRanger
#    Hypertable
#    HyperThriftConfig
#    HyperCommon
#    HyperDfsCmds
#    Hyperspace
#    HyperThrift
#    HyperTools

    ${SIGAR_LIBRARY}

    # Are all of these really needed ??
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
    ${Boost_REGEX_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

