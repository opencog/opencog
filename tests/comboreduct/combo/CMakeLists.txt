INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/ComboReduct
    ${CMAKE_SOURCE_DIR}/src/ComboReduct/ant_combo_vocabulary
)
LINK_DIRECTORIES(
    ${CMAKE_BINARY_DIR}/src/ComboReduct
)

ADD_CXXTEST(vertexUTest)
TARGET_LINK_LIBRARIES(vertexUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_CXXTEST(messageUTest)
TARGET_LINK_LIBRARIES(messageUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_CXXTEST(type_treeUTest)
TARGET_LINK_LIBRARIES(type_treeUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)

ADD_CXXTEST(variable_unifierUTest)
TARGET_LINK_LIBRARIES(variable_unifierUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)
ADD_CXXTEST(procedure_repositoryUTest)
TARGET_LINK_LIBRARIES(procedure_repositoryUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)
ADD_CXXTEST(evalUTest)
TARGET_LINK_LIBRARIES(evalUTest
	ComboReduct
    ${LADSUTIL_LIBRARY}
)
