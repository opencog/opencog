(define when-cop-q
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
                			(DefinedLinguisticRelationshipNode "_%atTime")
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
				(VariableNode "$verb")
				(WordNode "be")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-when-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)


(InheritanceLink (stv 1 .99) (ConceptNode "when-cop-q-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "when-cop-q-Rule") when-cop-q)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-when-cop-q-rule subj)
	(whencop-Q-rule (word-inst-get-word-str subj) (cog-name subj)
	)
)

