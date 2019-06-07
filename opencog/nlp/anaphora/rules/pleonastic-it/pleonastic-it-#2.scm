;; "It seems that"

(define pleonastic-it-#2
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$seems")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
        )
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
            (VariableNode "$target")
        )
    )
)
