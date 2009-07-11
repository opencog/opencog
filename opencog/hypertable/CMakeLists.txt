#SET(Boost_USE_STATIC_LIBS OFF)
#SET(Boost_USE_MULTITHREADED ON)
#FIND_PACKAGE(Boost 1.34 COMPONENTS program_options iostreams filesystem signals regex date_time thread REQUIRED)
#FIND_PACKAGE(EXPAT REQUIRED)

SET(HYPERTABLE_INCLUDE_DIRS, /opt/hypertable/0.9.2.4/include)

INCLUDE_DIRECTORIES(
#    ${Boost_INCLUDE_DIRS}
    /opt/hypertable/0.9.2.4/include
)

ADD_EXECUTABLE(hypertable-test test.cc)

ADD_LIBRARY(hypertabledb STATIC
	AtomspaceHTabler.cc
)
TARGET_LINK_LIBRARIES(hypertable-test
    hypertabledb
    /opt/hypertable/0.9.2.4/lib/libHypertable.a
    /home/jeremy/compile/opencog/atomspace/libatomspace.so
    /opt/hypertable/0.9.2.4/lib/libHyperComm.a
    /opt/hypertable/0.9.2.4/lib/libHyperDfsBroker.a
    /opt/hypertable/0.9.2.4/lib/libHyperRanger.a
    /opt/hypertable/0.9.2.4/lib/libHypertable.a
    /opt/hypertable/0.9.2.4/lib/libHyperThriftConfig.a
    /opt/hypertable/0.9.2.4/lib/libHyperCommon.a
    /opt/hypertable/0.9.2.4/lib/libHyperDfsCmds.a
    /opt/hypertable/0.9.2.4/lib/libHyperspace.a
    /opt/hypertable/0.9.2.4/lib/libHyperThrift.a
    /opt/hypertable/0.9.2.4/lib/libHyperTools.a
    /opt/hypertable/0.9.2.4/lib/libHyperCommon.a
    sigar-x86-linux
    curses
    log4cpp
    ${EXPAT_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
    ${Boost_SIGNALS_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

