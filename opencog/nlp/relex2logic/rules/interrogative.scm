(define interrogative
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$word-inst-node")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$parse-node")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$interp-node")
                (TypeNode "InterpretationNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$word-inst-node")
                (VariableNode "$parse-node")
            )
            ; WH-words will be POS-tagged with "interrogative"
            (InheritanceLink
                (VariableNode "$word-inst-node")
                (DefinedLinguisticConceptNode "interrogative")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        ; mark this as a question
        (InheritanceLink
            (VariableNode "$interp-node")
            (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
    )
)
