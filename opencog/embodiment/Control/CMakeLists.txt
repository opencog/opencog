ADD_SUBDIRECTORY (MessagingSystem)
ADD_SUBDIRECTORY (Predavese)
ADD_SUBDIRECTORY (PerceptionActionInterface)
ADD_SUBDIRECTORY (OperationalPetController)
ADD_SUBDIRECTORY (Procedure)
ADD_SUBDIRECTORY (PredicateUpdaters)

INCLUDE_DIRECTORIES (
    ${OPENCOG_SOURCE_DIR}/opencog/embodiment/AtomSpaceExtensions
    ${OPENCOG_SOURCE_DIR}/opencog
)

ADD_LIBRARY (Control
    EmbodimentConfig
    LoggerFactory
)
