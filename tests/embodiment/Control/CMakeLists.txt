ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (OperationalPetController)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (Procedure)
ADD_SUBDIRECTORY (Predavese)
ADD_SUBDIRECTORY (PredicateUpdaters)

INCLUDE_DIRECTORIES(
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/Control
)

LINK_DIRECTORIES(
    ${OPENCOG_BINARY_DIR}
    ${OPENCOG_BINARY_DIR}/opencog/embodiment/Control
)

#ADD_CXXTEST(SystemParametersUTest)
#TARGET_LINK_LIBRARIES(SystemParametersUTest Control ${LADSUTIL_LIBRARY})
#IF(WIN32)
#    TARGET_LINK_LIBRARIES(SystemParametersUTest winmm)
#ENDIF(WIN32)
