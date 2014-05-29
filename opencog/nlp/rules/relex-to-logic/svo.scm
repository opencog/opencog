; The focus set (the implicant) is equivalent to input-template of pln and
; criterium for r2l rules. The implicand is equivalent to the output-template
; of pln and the rule-functions of r2l.
(define svo
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
            )
            (ExecutionLink
                (GroundedSchemaNode "scm: pre-svo-rule")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Y")
                    (VariableNode "$Z")
                )
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svo-rule subj verb obj)
    (SVO-rule (word-inst-get-word-str subj) (cog-name subj)
              (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str obj) (cog-name obj)
    )
)


