;; Returning all number nodes for all WordInstanceNodes.

(define getAllNumberNodes
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$target")
                (TypeNode "WordInstanceNode")
            )
            (VariableNode "$number")
        )
        (ImplicationLink
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
)
