(define truthquery
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
            (InheritanceLink
                (VariableNode "$word-inst-node")
                (DefinedLinguisticConceptNode "truth-query")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        ; Mark this as a truth-query
        (InheritanceLink
            (VariableNode "$interp-node")
            (DefinedLinguisticConceptNode "TruthQuerySpeechAct"))
    )
)
