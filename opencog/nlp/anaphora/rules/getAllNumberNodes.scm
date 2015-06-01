;; Returning all number nodes for all WordInstanceNodes.

(define getAllNumberNodes
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (VariableNode "$number")
        )
        (AndLink
            (WordSequenceLink
                (VariableNode "$target")
                (VariableNode "$number")
            )
        )
        (ListLink
            (AnchorNode "CurrentResult")
            (WordSequenceLink
                (VariableNode "$target")
                (VariableNode "$number")
            )
        )
    )
)
