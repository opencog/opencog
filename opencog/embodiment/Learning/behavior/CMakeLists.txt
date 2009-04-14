ADD_LIBRARY(behavior
    BDTracker
    BE
    PAIWorldProvider
    BasicWorldProvider
    PredicateHandleSet
    BehaviorCategory
    BehaviorDescriptionMatcher
    CompositeBehaviorDescription
    EvaluationLinkSimilarityEvaluator
    BDRetriever
)

INCLUDE_DIRECTORIES (
    ${XERCESC_INCLUDE_DIR}
)

TARGET_LINK_LIBRARIES (behavior
	PAI
	util
	${XERCESC_LIBRARIES}
	MessagingSystem
	AtomSpaceExtensions
	atomspace
)
