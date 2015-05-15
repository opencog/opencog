(define gender
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
                (VariableNode "$gender")
                (TypeNode "DefinedLinguisticConceptNode")
            )
        )
        (ImplicationLink
            (AndLink
                (WordInstanceLink
                    (VariableNode "$noun")
                    (VariableNode "$a-parse")
                )
                (InheritanceLink
   			(VariableNode "$noun")
   			(VariableNode "$gender")
		)
            )
            (ExecutionOutputLink
          	  (GroundedSchemaNode "scm: pre-gender-rule")
           	      (ListLink
           	         (VariableNode "$noun")
           	         (VariableNode "$gender")
                	)
	     )
        )
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "gender-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "gender-Rule") gender)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-gender-rule noun gender)
    (gender-rule (word-inst-get-word-str noun) (cog-name noun)
		(cog-name gender)
    )
)


