(define quantity
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$noun")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$quantity")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$noun")
                    (VariableNode "$a-parse")
                )
		(WordInstanceLink
                    (VariableNode "$quantity")
                    (VariableNode "$a-parse")
                )
		(EvaluationLink
			(DefinedLinguisticRelationshipNode "_quantity")
			(ListLink
				(VariableNode "$noun")
				(VariableNode "$quantity")
			)
		)
            )
            (ExecutionOutputLink
          	  (GroundedSchemaNode "scm: pre-quantity-rule")
           	      (ListLink
           	         (VariableNode "$noun")
           	         (VariableNode "$quantity")
			
		)
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "quantity-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "quantity-Rule") quantity)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-quantity-rule noun quantity)
    (quantity-rule (word-inst-get-word-str noun)(cog-name noun)
		(word-inst-get-word-str quantity)(cog-name quantity)
    )
)


