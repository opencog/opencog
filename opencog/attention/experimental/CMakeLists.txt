# ExperimentalAttentionModule
ADD_LIBRARY(attention-experimental SHARED
            ExperimentalAttentionModule	
            ImportanceDiffusionBase
            AFImportanceDiffusionAgent
            WAImportanceDiffusionAgent
            RentCollectionBase
            AFRentCollectionAgent
            WARentCollectionAgent
)

ADD_DEPENDENCIES(attention-experimental attention-types)

TARGET_LINK_LIBRARIES(attention-experimental server ${ATOMSPACE_LIBRARIES})