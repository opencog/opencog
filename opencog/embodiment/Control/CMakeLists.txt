ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (Predavese)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (OperationalPetController)
ADD_SUBDIRECTORY (Procedure)
ADD_SUBDIRECTORY (PredicateUpdaters)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
)

ADD_LIBRARY (Control
    SystemParameters
    LoggerFactory
)
