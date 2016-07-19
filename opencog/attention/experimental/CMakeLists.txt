# ExperimentalAttentionModule
ADD_LIBRARY(attention-experimental SHARED
            ExperimentalAttentionModule
            ImportanceDiffusionBase
            AFImportanceDiffusionAgent
            WAImportanceDiffusionAgent
            RentCollectionBaseAgent
            AFRentCollectionAgent
            WARentCollectionAgent

            ForgettingAgent
            MinMaxSTIUpdatingAgent
            FocusBoundaryUpdatingAgent
            HebbianCreationAgent
            HebbianUpdatingAgent
)

ADD_DEPENDENCIES(attention-experimental attention-types)

TARGET_LINK_LIBRARIES(attention-experimental attention server ${ATOMSPACE_LIBRARIES})
