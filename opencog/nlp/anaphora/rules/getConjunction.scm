(define getConjunction
    (BindLink
        (ListLink
            (VariableNode "$target")
            (TypedVariableLink
                (VariableNode "$noun")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
                    (VariableNode "$target")
                )
                (PartOfSpeechLink
                    (VariableNode "$noun")
                    (DefinedLinguisticConceptNode "noun")
                )
               (EvaluationLink
                    (PrepositionalRelationshipNode "conj_and")
                    (ListLink
                        (VariableNode "$target")
                        (VariableNode "$noun")
                    )
               )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (VariableNode "$noun")
            )
        )
    )
)
