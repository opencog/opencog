; This rule is for "She wants John to kill you."  I hate these rules,
; which were here when I got here. There are five of them. There
; could be fifty.  There would need to be, to cover all cases.
; Somebody randomly chose five sentence-types to enshrine in these
; rules.  See my comment in the old relex2logic rule-file for more
; of an explanation of the problem.
; (AN June 2015)

(define todo2
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj1" "WordInstanceNode")
			(var-decl "$subj2" "WordInstanceNode")
			(var-decl "$verb1" "WordInstanceNode")
			(var-decl "$verb2" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$subj1-lemma" "WordNode")
			(var-decl "$subj2-lemma" "WordNode")
			(var-decl "$verb1-lemma" "WordNode")
			(var-decl "$verb2-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj1" "$a-parse")
			(word-in-parse "$subj2" "$a-parse")
			(word-in-parse "$verb1" "$a-parse")
			(word-in-parse "$verb2" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_subj" "$verb1" "$subj1")
			(dependency "_subj" "$verb2" "$subj2")
			(dependency "_obj" "$verb2" "$obj")
			(dependency "_to-do" "$verb1" "$verb2")
			(word-lemma "$subj1" "$subj1-lemma")
			(word-lemma "$subj2" "$subj2-lemma")
			(word-lemma "$verb1" "$verb1-lemma")
			(word-lemma "$verb2" "$verb2-lemma")
			(word-lemma "$obj" "$obj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: to-do-rule-2")
			(ListLink
				(VariableNode "$subj1-lemma")
				(VariableNode "$subj1")
				(VariableNode "$subj2-lemma")
				(VariableNode "$subj2")
				(VariableNode "$verb1-lemma")
				(VariableNode "$verb1")
				(VariableNode "$verb2-lemma")
				(VariableNode "$verb2")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
			)
		)
	)
)
