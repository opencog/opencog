; This rule is for when copula questions, such as "When is the meeting."
; Well actually that's not a copula but let's not get into that, okay. You get the idea anyway.
; (AN June 2015)

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
   (ListLink
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-when-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-when-cop-q-rule subj)
 (ListLink
	(whencop-Q-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
	)
 )
)
