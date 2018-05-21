ADD_CXXTEST(AttentionParamQueryUTest)
          
TARGET_LINK_LIBRARIES(AttentionParamQueryUTest
    attention
    ${COGUTIL_LIBRARY}
    )
