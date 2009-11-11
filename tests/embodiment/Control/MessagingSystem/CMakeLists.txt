CMAKE_POLICY(SET CMP0005 OLD)
ADD_DEFINITIONS(-D_REENTRANT -DTEST_CONFIG_FILE=\\"${CMAKE_CURRENT_SOURCE_DIR}/config.cfg\\")

SET(CMAKE_BUILD_TYPE Debug)

ADD_CXXTEST(RouterMessageUTest)
TARGET_LINK_LIBRARIES(RouterMessageUTest MessagingSystem)

ADD_CXXTEST(FileMessageCentralUTest)
TARGET_LINK_LIBRARIES(FileMessageCentralUTest MessagingSystem)

ADD_CXXTEST(MemoryMessageCentralUTest)
TARGET_LINK_LIBRARIES(MemoryMessageCentralUTest MessagingSystem)

ADD_CXXTEST(StringMessageUTest)
TARGET_LINK_LIBRARIES(StringMessageUTest MessagingSystem)

