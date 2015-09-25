; This rule is for where questions with verb "be"
; such as "Where is the meeting?"
;(AN June 2015)

(define where-cop-q
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
                			(DefinedLinguisticRelationshipNode "_%atLocation")
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
			(GroundedSchemaNode "scm: pre-where-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
   )
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-where-cop-q-rule subj)
 (ListLink
	(wherecop-Q-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
	)
 )
)
