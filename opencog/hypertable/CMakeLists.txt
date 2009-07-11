#SET(Boost_USE_STATIC_LIBS OFF)
#SET(Boost_USE_MULTITHREADED ON)
#FIND_PACKAGE(Boost 1.34 COMPONENTS program_options iostreams filesystem signals regex date_time thread REQUIRED)
#FIND_PACKAGE(EXPAT REQUIRED)

#SET(HYPERTABLE_INCLUDE_DIRS, /opt/hypertable/0.9.2.4/include)

INCLUDE_DIRECTORIES(
#    ${Boost_INCLUDE_DIRS}
    /opt/hypertable/0.9.2.4/include
#    /home/jeremy/hypertable-0.9.3/0.9.2.4/include
)

ADD_EXECUTABLE(hypertable-test test.cc)

ADD_LIBRARY(hypertabledb STATIC
	AtomspaceHTabler.cc
)
TARGET_LINK_LIBRARIES(hypertable-test
    hypertabledb
    /home/jeremy/compile/opencog/atomspace/libatomspace.so
    /opt/hypertable/0.9.2.4/lib/libHyperComm.so
    /opt/hypertable/0.9.2.4/lib/libHyperDfsBroker.so
    /opt/hypertable/0.9.2.4/lib/libHyperRanger.so
    /opt/hypertable/0.9.2.4/lib/libHypertable.so
    /opt/hypertable/0.9.2.4/lib/libHyperThriftConfig.so
    /opt/hypertable/0.9.2.4/lib/libHyperCommon.so
    /opt/hypertable/0.9.2.4/lib/libHyperDfsCmds.so
    /opt/hypertable/0.9.2.4/lib/libHyperspace.so
    /opt/hypertable/0.9.2.4/lib/libHyperThrift.so
    /opt/hypertable/0.9.2.4/lib/libHyperTools.so

#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperComm.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperDfsBroker.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperRanger.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHypertable.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperThriftConfig.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperCommon.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperDfsCmds.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperspace.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperThrift.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperTools.so
#/home/jeremy/hypertable-0.9.3/0.9.2.4/lib/libHyperCommon.so    

    sigar-x86-linux
    curses
    log4cpp
    ${EXPAT_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_IOSTREAMS_LIBRARY}
    ${Boost_SIGNALS_LIBRARY} ${Boost_REGEX_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

