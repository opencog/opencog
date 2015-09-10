ADD_LIBRARY(PetaverseMOSES
	moses-learning
)

TARGET_LINK_LIBRARIES(PetaverseMOSES
    moses
    ${COGUTIL_LIBRARY}
    comboreduct
    FitnessEstimator
    AvatarComboVocabulary
)
