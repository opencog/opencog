ADD_LIBRARY(minerutestutils SHARED
  MinerUTestUtils
)

TARGET_LINK_LIBRARIES(minerutestutils
  miner
  guile-miner
)

ADD_CXXTEST(MinerUTest)
TARGET_LINK_LIBRARIES(MinerUTest
  minerutestutils
)
