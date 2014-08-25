;; "It seems that"

(define pleonastic-it-#2
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$seems")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
                    (VariableNode "$target")
                )
                (LemmaLink
                    (VariableNode "$seems")
                    (WordNode "seem")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_expl")
                    (ListLink
                        (VariableNode "$seems")
                        (VariableNode "$target")
                    )
                )
                (EvaluationLink
                    (PrepositionalRelationshipNode "that")
                    (ListLink
                        (VariableNode "$seems")
                        (VariableNode "$word")
                    )
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (AnchorNode "Matched")
            )
        )
    )
)
