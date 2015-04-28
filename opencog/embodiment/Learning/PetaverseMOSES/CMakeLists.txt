ADD_LIBRARY(PetaverseMOSES
	moses-learning
)

TARGET_LINK_LIBRARIES(PetaverseMOSES
    moses
    cogutil
    comboreduct
    FitnessEstimator
    AvatarComboVocabulary
)
