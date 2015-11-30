INCLUDE_DIRECTORIES(
    ${COGUTIL_INCLUDE_DIR}
    ${ATOMSPACE_INCLUDE_DIR}
    #${CMAKE_BINARY_DIR}
)

LINK_DIRECTORIES(
    ${ATOMSPACE_LIBPATH}
)

# The following is more appropriate but only works when cmake is run.
#FILE(COPY test-rbs.scm DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
CONFIGURE_FILE(test-rbs.scm test-rbs.scm COPYONLY)
IF(HAVE_GUILE)
    ADD_CXXTEST(ActionUTest)
    TARGET_LINK_LIBRARIES(ActionUTest
        ${ATOMSPACE_LIBRARY}
        ${ATOMSPACE_smob_LIBRARY}
        ${COGUTIL_LIBRARY}
        ${ATOMSPACE_ruleengine_LIBRARY}
        planning
    )
ENDIF(HAVE_GUILE)
