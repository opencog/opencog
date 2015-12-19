; This rule is for copula yes/no questions, such as . . . "Are you my mommy?"
; (AN June 2015)

(define copula-ynq
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_subj" "$verb" "$subj")
			(dependency "_obj" "$verb" "$obj")
			(LemmaLink
				(VariableNode "$verb")
				(WordNode "be")
			)
			(InheritanceLink
				(VariableNode "$verb")
				(DefinedLinguisticConceptNode "truth-query")
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-copula-ynq-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$obj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-copula-ynq-rule subj obj)
	(cop-ynQ-rule (cog-name (word-inst-get-lemma subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma obj)) (cog-name obj)
	)
)
