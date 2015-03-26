;; Returning all WordInstanceNode

(define getWords
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$parse node")
                (TypeNode "ParseNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$target")
                    (VariableNode "$parse node")
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (VariableNode "$target")
            )
        )
    )
)
