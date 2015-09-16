; This the rule for subject-verb-object sentences, such as
; "Johnny ate the dog."
; (AN June 2015)

(define svo
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$W")
                (TypeNode "WordInstanceNode")
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
            (AbsentLink
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_iobj")
                    (ListLink
                        (VariableNode "$Y")
                        (VariableNode "$W")
                    )
                )
            )
		)
       (ListLink
        (ExecutionOutputLink
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

;;(InheritanceLink (stv 1 .99) (ConceptNode "SVO-Rule") (ConceptNode "Rule"))

;;(ReferenceLink (stv 1 .99) (ConceptNode "SVO-Rule") svo)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svo-rule subj verb obj)
(ListLink
    (SVO-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
              (cog-name (word-inst-get-lemma verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
    ))
)
