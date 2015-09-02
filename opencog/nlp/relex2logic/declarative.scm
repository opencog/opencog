(define declarative
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
            (LemmaLink
                (VariableNode "$word-inst-node")
                (WordNode ".")
            )
            (PartOfSpeechLink
                (VariableNode "$word-inst-node")
                (DefinedLinguisticConceptNode "punctuation")
            )
            (WordInstanceLink
                (VariableNode "$word-inst-node")
                (VariableNode "$parse-node")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        (ListLink
            (ExecutionOutputLink
                (GroundedSchemaNode "scm: pre-decl-rule")
                (ListLink
                    (VariableNode "$interp-node")
                )
            )
        )
    )
)

(define (pre-decl-rule int-index)
    (ListLink
        (declarative-rule int-index)
    )
)
