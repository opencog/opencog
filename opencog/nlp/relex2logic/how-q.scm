; NB: in order to make this rule work, I have had to retain the classification of "how" as a "PrepositionalRelationshipNode" because that is 
; what Relex is putting into the atomspace.  It makes no sense, but since I don't even know where is the code that generates the relex atoms, 
; and changing it would probably muck up something else, so I'm just leaving it!

(define how-q
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
				(VariableNode "$qVar")
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
					(VariableNode "$qVar")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
                    			(PrepositionalRelationshipNode "how")
                    			(ListLink
                        			(VariableNode "$verb")
                        			(VariableNode "$qVar")
                    			)					
                		)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-how-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
))

(InheritanceLink (stv 1 .99) (ConceptNode "how-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "how-q-Rule") how-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-how-q-rule verb)
	(how-rule (word-inst-get-word-str verb) (cog-name verb)
	)
)

