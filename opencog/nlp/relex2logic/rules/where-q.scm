; This rule is for where questions without the verb "be"
; such as "Where do you go every night?"
; (AN June 2015)

(define where-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(dependency "_%atLocation" "$verb" "$qVar")
			(word-lemma "$verb" "$verb-lemma")
			(Absent (Lemma (Variable "$verb") (Word "be")))
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: where-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
			)
		)
	)
)
