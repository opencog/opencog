(define advmod
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$verb")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$adv")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$verb")
                    (VariableNode "$a-parse")
                )
                (WordInstanceLink
                    (VariableNode "$adv")
                    (VariableNode "$a-parse")
                )
                (EvaluationLink
                    (DefinedLinguisticRelationshipNode "_advmod")
                    (ListLink
                        (VariableNode "$verb")
                        (VariableNode "$adv")
                    )
                )
            )
            (ExecutionOutputLink
           	   (GroundedSchemaNode "scm: pre-advmod-rule")
           	      (ListLink
           	         (VariableNode "$verb")
           	         (VariableNode "$adv")
                )
            )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "advmod-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "advmod-Rule") advmod)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-advmod-rule verb adv)
    (advmod-rule (word-inst-get-word-str verb) (cog-name verb)
              (word-inst-get-word-str adv) (cog-name adv)
    )
)


