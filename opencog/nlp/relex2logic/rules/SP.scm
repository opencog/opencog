; This rule is for the main predicate in predicate-adjective sentences
; such as "You are very young, sir."
; (AN June 2015)


(define SP
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$predadj" "WordInstanceNode")
			(var-decl "$subj-lemma" "WordNode")
			(var-decl "$predadj-lemma" "WordNode")
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$predadj")
				(VariableNode "$a-parse")
			)
			(LemmaLink
				(VariableNode "$subj")
				(VariableNode "$subj-lemma")
			)
			(LemmaLink
				(VariableNode "$predadj")
				(VariableNode "$predadj-lemma")
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_predadj")
				(ListLink
					(VariableNode "$subj")
					(VariableNode "$predadj")
				)
			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: SV-rule")
			(ListLink
				(VariableNode "$subj-lemma")
				(VariableNode "$subj")
				(VariableNode "$predadj-lemma")
				(VariableNode "$predadj")
			)
		)
	)
)
