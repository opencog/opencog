ADD_LIBRARY(PetaverseMOSES
	moses-learning
)

TARGET_LINK_LIBRARIES(PetaverseMOSES
    moses
    util
    comboreduct
    FitnessEstimator
    AvatarComboVocabulary
)
