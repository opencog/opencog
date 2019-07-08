ADD_LIBRARY(minerutestutils SHARED
  MinerUTestUtils
)

TARGET_LINK_LIBRARIES(minerutestutils
  miner
  guile-miner
  ${URE_LIBRARIES}
)

ADD_CXXTEST(MinerUTest)
TARGET_LINK_LIBRARIES(MinerUTest
  minerutestutils
)

ADD_CXXTEST(SurprisingnessUTest)
TARGET_LINK_LIBRARIES(SurprisingnessUTest
  minerutestutils
)
