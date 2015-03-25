;; Checking if the target is the word "it".

(define isIt
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
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
                    (WordNode "it")
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (AnchorNode "Matched")
            )
        )
    )
)
