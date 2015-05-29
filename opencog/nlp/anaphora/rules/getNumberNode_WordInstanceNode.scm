;; Returning the NumberNode of a WordInstanceNode.

(define getNumberNode_WordInstanceNode
    (BindLink
        (VariableList
            (VariableNode "$target")
            (VariableNode "$number")
        )
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
