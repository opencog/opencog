(define getPronouns
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
                 (NotLink
                    (ReferenceLink
                        (VariableNode "$word-inst")
                        (WordNode "we")
                    )
                 )
                 (NotLink
                    (LemmaLink
                        (VariableNode "$word-inst")
                        (WordNode "I")
                    )
                 )
                 (NotLink
                    (LemmaLink
                        (VariableNode "$word-inst")
                        (WordNode "me")
                    )
                 )
            )
            (ListLink
                 (AnchorNode "Recent Unresolved references")
                 (VariableNode "$word-inst")
            )
        )
    )
)