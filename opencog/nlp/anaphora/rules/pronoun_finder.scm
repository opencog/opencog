(define pronoun-finder
    (BindLink
        (ListLink
            (VariableNode "$sent")
            (VariableNode "$parse")
            (VariableNode "$word-inst")
        )
        (ImplicationLink
            (AndLink
                 (ListLink (AnchorNode "# New Parsed Sentence") (VariableNode "$sent"))
                 (ParseLink (VariableNode "$parse") (VariableNode "$sent"))
                 (WordInstanceLink (VariableNode "$word-inst") (VariableNode "$parse"))
                 (InheritanceLink
                    (VariableNode "$word-inst")
                    (DefinedLinguisticConceptNode "pronoun")
                 )
            )
            (ListLink
                 (AnchorNode "Recent Unresolved references")
                 (VariableNode "$word-inst")
            )
        )
    )
)