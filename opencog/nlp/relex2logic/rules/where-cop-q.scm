; This rule is for where questions with verb "be"
; such as "Where is the meeting?"
;(AN June 2015)

(define where-cop-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
			(var-decl "$subj" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(word-in-parse "$subj" "$a-parse")
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
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-where-cop-q-rule")
			(ListLink
				(VariableNode "$subj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-where-cop-q-rule subj)
	(wherecop-Q-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj))
)
