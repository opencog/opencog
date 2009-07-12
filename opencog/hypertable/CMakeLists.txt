#SET(Boost_USE_STATIC_LIBS OFF)
#SET(Boost_USE_MULTITHREADED ON)
#FIND_PACKAGE(Boost 1.34 COMPONENTS program_options iostreams filesystem signals regex date_time thread REQUIRED)
#FIND_PACKAGE(EXPAT REQUIRED)

#SET(HYPERTABLE_INCLUDE_DIRS, /opt/hypertable/0.9.2.4/include)

INCLUDE_DIRECTORIES(
#    ${Boost_INCLUDE_DIRS}
    /opt/hypertable/0.9.2.4/include
    /usr/hypertable/0.9.2.4/include
    /usr/local/hypertable/0.9.2.4/include
#    /home/jeremy/hypertable-0.9.3/0.9.2.4/include
)

ADD_LIBRARY(hypertabledb STATIC
	AtomspaceHTabler.cc
)

ADD_EXECUTABLE(hypertable-test test.cc)

LINK_DIRECTORIES(
    /usr/local/hypertable/0.9.2.4/lib
)

TARGET_LINK_LIBRARIES(hypertable-test
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

