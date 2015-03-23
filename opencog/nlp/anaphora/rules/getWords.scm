;; Returning all WordInstanceNode

(define getWords
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$parse node")
                (VariableTypeNode "ParseNode")
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
