;; target is "much" and there does not exist "of" after it.

(define pre-process-#5
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
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
                    (VariableNode "$target")
                    (WordNode "much")
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
                ;; Adding a missing tag
                (InheritanceLink
                    (VariableNode "$target")
                    (DefinedLinguisticConceptNode "singular")
                )
                (AnchorNode "CurrentResult")
                (AnchorNode "Matched")
            )
        )
    )
)
