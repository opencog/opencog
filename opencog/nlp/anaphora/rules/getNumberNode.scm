(define getNumberNode
    (BindLink
        (ListLink
            (VariableNode "$target")
            (VariableNode "$number")
        )
        (ImplicationLink
            (AndLink
                (ListLink
                    (AnchorNode "currentTarget")
                    (VariableNode "$target")
                )
                (WordSequenceLink
                    (VariableNode "$target")
                    (VariableNode "$number")
                )
            )
            (ListLink
                (AnchorNode "currentResult")
                (VariableNode "$number")
            )
        )
    )
)
