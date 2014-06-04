(define be-inheritance
    (BindLink
        (ListLink
            (TypedVariableLink
                (VariableNode "$a-parse")
                (VariableTypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$X")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Y")
                (VariableTypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Z")
                (VariableTypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$X")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$Y")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$Z")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_subj")
                    (ListLink
                        (VariableNode "$Y")
                        (VariableNode "$X")
                    )
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_obj")
                    (ListLink
                        (VariableNode "$Y")
                        (VariableNode "$Z")
                    )
                )
                (LemmaLink
                    (VariableNode "$Y")
                    (WordNode "be")
                )
            )
            (ExecutionLink
                (GroundedSchemaNode "scm: pre-be-inheritance-rule")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Z")
                )
            )
        )
    )
)


; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-be-inheritance-rule subj obj)
    (be-inheritance-rule (word-inst-get-word-str subj) (cog-name subj)
              (word-inst-get-word-str obj) (cog-name obj)
    )
)

