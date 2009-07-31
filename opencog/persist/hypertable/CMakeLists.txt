
INCLUDE_DIRECTORIES(
	${HYPERTABLE_INCLUDE_DIRS}
#    /home/jeremy/hypertable-0.9.3/0.9.2.4/include
)

ADD_LIBRARY(hypertabledb STATIC
	AtomspaceHTabler.cc
)

ADD_EXECUTABLE(hypertable-test test.cc)

# LINK_DIRECTORIES(
#     /usr/local/hypertable/0.9.2.4/lib
# )

TARGET_LINK_LIBRARIES(hypertable-test
    ${HYPERTABLE_LIBRARIES}
    hypertabledb

    HyperComm
    HyperDfsBroker
    HyperRanger
    Hypertable
    HyperThriftConfig
    HyperCommon
    HyperDfsCmds
    Hyperspace
    HyperThrift
    HyperTools

#    sigar-x86-linux
    sigar-x86_64-linux
    curses
    log4cpp
    ${EXPAT_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
    ${Boost_SIGNALS_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

