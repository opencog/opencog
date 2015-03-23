;; Returning the NumberNode of a ParseNode.

(define getNumberNode_ParseNode
    (BindLink
        (ListLink
            (VariableNode "$target")
            (VariableNode "$sentence")
            (VariableNode "$number")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
                    (VariableNode "$target")
                )
                (ParseLink
                    (VariableNode "$target")
                    (VariableNode "$sentence")
                )
                (SentenceSequenceLink
                    (VariableNode "$sentence")
                    (VariableNode "$number")
                )
            )
            (ListLink
                (AnchorNode "CurrentResult")
                (VariableNode "$number")
            )
        )
    )
)
