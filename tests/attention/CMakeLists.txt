ADD_CXXTEST(AttentionParamQueryUTest)
          
TARGET_LINK_LIBRARIES(AttentionParamQueryUTest
    attention
    ${ATOMSPACE_LIBRARIES}
    ${ATOMSPACE_query_LIBRARY}
    ${ATOMSPACE_atombase_LIBRARY}
    ${COGUTIL_LIBRARY}
    )
