(define why-cop-q
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
			(TypedVariableLink
				(VariableNode "$subj")
				(TypeNode "WordInstanceNode")
			)
		)
		(AndLink	
			(WordInstanceLink
				(VariableNode "$verb")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$qVar")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(EvaluationLink
                			(DefinedLinguisticRelationshipNode "_%because")
                			(ListLink
                    			(VariableNode "$verb")
                    			(VariableNode "$qVar")
                			)			
            		)
			(ChoiceLink
				(AndLink					
					(EvaluationLink
            	    				(DefinedLinguisticRelationshipNode "_subj")
            	    				(ListLink
            	        				(VariableNode "$verb")
            	        				(VariableNode "$subj")
            	    				)			
            				)
					(LemmaLink
						(VariableNode "$verb")
						(WordNode "be")
					)
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_predadj")
					(ListLink
						(VariableNode "$subj")
						(VariableNode "$verb")
					)
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_psubj")
					(ListLink
						(VariableNode "$verb")
						(VariableNode "$subj")
					)
				)
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-why-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)


(InheritanceLink (stv 1 .99) (ConceptNode "why-cop-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "why-cop-q-Rule") why-cop-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-why-cop-q-rule subj)
	(whycop-Q-rule (word-inst-get-word-str subj) (cog-name subj)
	)
)

