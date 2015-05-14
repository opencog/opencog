(define howpredadj1-q
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$subj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$qVar")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$verb")
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
					(VariableNode "$subj")
					(VariableNode "$a-parse")
				)
				(WordInstanceLink
					(VariableNode "$qVar")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
                    			(DefinedLinguisticRelationshipNode "_%how")
                    			(ListLink
                        			(VariableNode "$verb")
                        			(VariableNode "$qVar")
                    			)					
                		)
				(EvaluationLink
                    			(DefinedLinguisticRelationshipNode "_subj")
                    			(ListLink
                        			(VariableNode "$verb")
                        			(VariableNode "$subj")
                    			)					
                		)
				(LemmaLink
					(WordInstanceNode "$verb")
					(WordNode "be")
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-howpredadj1-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
))

(InheritanceLink (stv 1 .99) (ConceptNode "howpredadj1-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "howpredadj1-q-Rule") howpredadj1-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-howpredadj1-q-rule subj)
	(howpredadj-Q-rule (word-inst-get-word-str subj) (cog-name subj)
	)
)

