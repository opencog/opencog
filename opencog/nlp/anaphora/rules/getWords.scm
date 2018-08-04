;; Returning all WordInstanceNode

(define getWords
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$parse node")
                (TypeNode "ParseNode")
            )
        )
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
