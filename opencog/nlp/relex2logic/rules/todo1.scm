; This rule is for "She wants to help John."  I hate these rules, which
; were here when I got here. There are five of them. There could be fifty.
; There would need to be, to cover all cases.  Somebody randomly chose
; five sentence-types to enshrine in these rules.  See my comment in the
; old relex2logic rule-file for more of an explanation of the problem.
; (AN June 2015)

(define todo1
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$subj2" "WordInstanceNode")
			(var-decl "$verb1" "WordInstanceNode")
			(var-decl "$verb2" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$verb1-lemma" "WordNode")
			(var-decl "$verb2-lemma" "WordNode")
			(var-decl "$obj-lemma" "WordNode")
		)
		(AndLink
			(word-in-parse "$subj" "$a-parse")
			(word-in-parse "$verb1" "$a-parse")
			(word-in-parse "$verb2" "$a-parse")
			(word-in-parse "$obj" "$a-parse")
			(dependency "_subj" "$verb1" "$subj")
			(dependency "_obj" "$verb2" "$obj")
			(dependency "_to-do" "$verb1" "$verb2")
			(AbsentLink
				(dependency "_subj" "$verb2" "$subj2")
			)
			(word-lemma "$subj" "$subj-lemma")
			(word-lemma "$verb1" "$verb1-lemma")
			(word-lemma "$verb2" "$verb2-lemma")
			(word-lemma "$obj" "$obj-lemma")
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: to-do-rule-1")
			(ListLink
				(VariableNode "$verb1-lemma")
				(VariableNode "$verb1")
				(VariableNode "$verb2-lemma")
				(VariableNode "$verb2")
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$obj-lemma")
				(VariableNode "$obj")
			)
		)
	)
)
