(define possessive
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
	                (VariableNode "$possessive")
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
                    			(VariableNode "$possessive")
                    			(VariableNode "$a-parse")
                		)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_poss")
					(ListLink
						(VariableNode "$noun")
						(VariableNode "$possessive")
					)
				)
			)
            (ExecutionOutputLink
          	  (GroundedSchemaNode "scm: pre-possessive-rule")
           	      (ListLink
           	         (VariableNode "$noun")
           	         (VariableNode "$possessive")	
			)
        	)
	)
    )
)

(InheritanceLink (stv 1 .99) (ConceptNode "possessive-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "possessive-Rule") possessive)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-possessive-rule noun possessive)
    (possessive-rule (word-inst-get-word-str noun)(cog-name noun)
		(word-inst-get-word-str possessive)(cog-name possessive)
    )
)


