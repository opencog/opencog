;; Returning all ParseNodes in the atomspace.

(define getAllParseNodes
    (BindLink
        (ListLink
            (VariableNode "$parse node")
            (VariableNode "$sentence")
        )
        (ImplicationLink
            (ParseLink
                    (VariableNode "$parse node")
                    (VariableNode "$sentence")
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (VariableNode "$parse node")
            )
        )
    )
)
