(define imperative
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
                (DefinedLinguisticConceptNode "imperative")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pre-imp-rule")
                (ListLink
                    (VariableNode "$interp-node")
                )
            )
        )
    )
)

(define (pre-imp-rule int-index)
    (ListLink
        (imperative-rule int-index)
    )
)
