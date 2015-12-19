; This rule is for which-objects as in
; "Which book did you read?"
; (AN June 2015)

(define whichobjQ
    (BindLink
        (VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$subj")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$verb")
                (VariableNode "$a-parse")
            )
            (WordInstanceLink
                (VariableNode "$obj")
                (VariableNode "$a-parse")
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_subj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$subj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_obj")
                (ListLink
                    (VariableNode "$verb")
                    (VariableNode "$obj")
                )
            )
            (EvaluationLink
                (DefinedLinguisticRelationshipNode "_det")
                (ListLink
                    (VariableNode "$obj")
                    (VariableNode "$qVar")
                )
            )
            (InheritanceLink
                (VariableNode "$qVar")
                (DefinedLinguisticConceptNode "which")
            )
        )
        (ExecutionOutputLink
              (GroundedSchemaNode "scm: pre-whichobjQ-rule")
                 (ListLink
                    (VariableNode "$subj")
                    (VariableNode "$verb")
                    (VariableNode "$obj")
            )
        )
    )
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-whichobjQ-rule subj verb obj)
    (whichobjQ-rule (cog-name (word-inst-get-lemma  obj)) (cog-name obj)
              (cog-name (word-inst-get-lemma  verb)) (cog-name verb)
              (cog-name (word-inst-get-lemma subj)) (cog-name subj)
    )
)
