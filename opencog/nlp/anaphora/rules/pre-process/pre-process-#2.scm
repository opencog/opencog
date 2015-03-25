;; target is a number and there does not exist "of" after it.

(define pre-process-#2
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
                    (VariableNode "$target")
                )
                (InheritanceLink
                    (VariableNode "$target")
                    (DefinedLinguisticConceptNode "numeric")
                )
                (NotLink
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
                (AnchorNode "Matched")
            )
        )
    )
)
