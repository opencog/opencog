;; Returning all ParseNodes in the atomspace.

(define getAllParseNodes
    (BindLink
        (VariableList
            (VariableNode "$parse node")
            (VariableNode "$sentence")
        )
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
