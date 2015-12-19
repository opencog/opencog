; This rule is for sentences with an indirect object, such as:
; "I gave the dog some LSD."  "I sang a song to her."
; For indirect objects hanging off a preposition, see SVIO2.
; (AN June 2015)


(define SVIO1
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
			(var-decl "$iobj" "WordInstanceNode")
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
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_subj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$subj")
				)
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_obj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$obj")
				)
			)
			(EvaluationLink
				(DefinedLinguisticRelationshipNode "_iobj")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$iobj")
				)
			)
			(LemmaLink
				(VariableNode "$subj")
				(VariableNode "$subj-lemma")
			)
			(LemmaLink
				(VariableNode "$verb")
				(VariableNode "$verb-lemma")
			)
			(LemmaLink
				(VariableNode "$obj")
				(VariableNode "$obj-lemma")
			)
			(LemmaLink
				(VariableNode "$iobj")
				(VariableNode "$iobj-lemma")
			)
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
