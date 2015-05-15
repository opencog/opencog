(define negative
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$a-parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$word")
                (TypeNode "WordInstanceNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$word")
                    (VariableNode "$a-parse")
                )
                (InheritanceLink
   			(VariableNode "$word")
   			(DefinedLinguisticConceptNode "negative")
		)
            )
            (ExecutionOutputLink
          	  (GroundedSchemaNode "scm: pre-negative-rule")
           	      (ListLink
           	         (VariableNode "$word")
			)
		)
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "negative-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "negative-Rule") negative)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-negative-rule word)
    (negative-rule (word-inst-get-word-str word)(cog-name word)
    )
)


