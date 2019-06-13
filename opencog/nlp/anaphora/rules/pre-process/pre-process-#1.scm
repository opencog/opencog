;; target is a pronoun and there does not exist "of" after it.

(define pre-process-#1
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
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
            (InheritanceLink
                (VariableNode "$target")
                (DefinedLinguisticConceptNode "pronoun")
            )
            (AbsentLink
                (EvaluationLink
                    (PrepositionalRelationshipNode "of")
                    (ListLink
                        (VariableNode "$target")
                        (VariableNode "$word")
                    )
                )
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$target")
        )
    )
)
