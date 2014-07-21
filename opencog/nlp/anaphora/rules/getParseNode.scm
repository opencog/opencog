;; Given a WordInstanceNode, returns a ParseNode associated with it

(define getParseNode
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$parse node")
                (VariableTypeNode "ParseNode")
            )
            (VariableNode "$target")

        )
        (ImplicationLink
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
)
