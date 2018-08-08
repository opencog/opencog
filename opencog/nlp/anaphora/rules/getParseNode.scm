;; Given a WordInstanceNode, returns a ParseNode associated with it

(define getParseNode
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$parse node")
                (TypeNode "ParseNode")
            )
            (VariableNode "$target")

        )
        (AndLink
            (ListLink
                (AnchorNode "CurrentTarget")
                (VariableNode "$target")
            )
            (WordInstanceLink
                (VariableNode "$target")
                (VariableNode "$parse node")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (VariableNode "$parse node")
        )
    )
)
