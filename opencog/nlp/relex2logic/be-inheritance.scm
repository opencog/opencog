(define be-inheritance
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$X")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Y")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$Z")
                (TypeNode "WordInstanceNode")
            )
        )
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
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pre-be-inheritance-rule")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$Z")
            )
        )
    )
)

(InheritanceLink (stv 1 .99 ) (ConceptNode "BE-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99 ) (ConceptNode "BE-Rule") be-inheritance)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-be-inheritance-rule subj obj)
    (be-inheritance-rule (word-inst-get-word-str subj) (cog-name subj)
              (word-inst-get-word-str obj) (cog-name obj)
    )
)

