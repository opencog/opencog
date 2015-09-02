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
            (InheritanceLink
                (VariableNode "$word-inst-node")
                (DefinedLinguisticConceptNode "interrogative")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pre-interr-rule")
                (ListLink
                    (VariableNode "$interp-node")
                )
            )
        )
    )
)

(define (pre-interr-rule int-index)
    (begin
        (display "interrogative !!!\n")
        (ListLink
            (interrogative-rule int-index)
        )
    )
)
