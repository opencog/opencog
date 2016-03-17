; This rule is for snagging some of the indirect objects that aren't
; lucky enough to get the _iobj relation from relex.  So, instead
; they're just object of the preposition "to". Poor things.
; Example: "I sent the money to your boss."
; (AN June 2015)

(define SVIO2
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
			(var-decl "$to" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
			(var-decl "$iobj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(word-in-parse "$iobj" "$a-parse")
			(word-in-parse "$to" "$a-parse")
			(Lemma (Variable "$to") (Word "to"))
			(dependency "_subj" "$subj" "$verb")
			(dependency "_obj" "$obj" "$verb")
			(dependency "_pobj" "$iobj" "$to")
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb" "$verb-lemma")
			(word-lemma "$obj" "$obj-lemma")
			(word-lemma "$iobj" "$iobj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SVIO-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$verb-lemma")
				(VariableNode "$verb")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
				(VariableNode "$iobj-lemma")
				(VariableNode "$iobj")
			)
		)
	)
)
