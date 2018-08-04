; This rule is for subject-verb sentences, such as
; "The Lord giveth and the lord taketh away." or "You suck."
; (AN June 2015)

(define SV
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(dependency "_subj" "$verb" "$subj")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SV-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
			)
		)
	)
)
