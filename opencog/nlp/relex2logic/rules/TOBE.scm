; This rule is for adjectival intensional complements of certain verbs,
; such as in "He seems to be happy." I'm not sure how it gets assigned;
; it's one of those random things that was here when I got here; I want
; to point out that there are also uses of these verbs with other
; word-types following them, such as "He seems like a nice guy." and
; "He seems to run the show." So, this rule should probably be handled
; by something more gerneral, but I haven't gotten around to it yet.
; (AN June 2015)


(define TOBE
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$adj" "WordInstanceNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$adj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$adj" "$a-parse")
			(dependency "_to-be" "$verb" "$adj")
			(dependency "_subj" "$verb" "$subj")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$adj" "$adj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: to-be-rule")
			(ListLink
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$adj-lemma")
				(VariableNode "$adj")
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
			)
		)
	)
)
