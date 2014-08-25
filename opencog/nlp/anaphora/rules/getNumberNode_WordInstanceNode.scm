;; Returning the NumberNode of a WordInstanceNode.

(define getNumberNode_WordInstanceNode
    (BindLink
        (ListLink
            (VariableNode "$target")
            (VariableNode "$number")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "CurrentTarget")
                    (VariableNode "$target")
                )
                (WordSequenceLink
                    (VariableNode "$target")
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
