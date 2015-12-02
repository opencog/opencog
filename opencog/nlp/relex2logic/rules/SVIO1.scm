; This rule is for sentences with an indirect object, such as:
; "I gave the dog some LSD."  "I sang a song to her."
; For indirect objects hanging off a preposition, see SVIO2.
; (AN June 2015)


(define SVIO1
	(BindLink
		(VariableList
			(TypedVariableLink
				(VariableNode "$a-parse")
				(TypeNode "ParseNode")
			)
			(TypedVariableLink
				(VariableNode "$subj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$verb")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$obj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$iobj")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$subj-lemma")
				(TypeNode "WordNode")
			)
			(TypedVariableLink
				(VariableNode "$verb-lemma")
				(TypeNode "WordNode")
			)
			(TypedVariableLink
				(VariableNode "$obj-lemma")
				(TypeNode "WordNode")
			)
			(TypedVariableLink
				(VariableNode "$iobj-lemma")
				(TypeNode "WordNode")
			)
		)
		(AndLink
			(WordInstanceLink
				(VariableNode "$subj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$verb")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$obj")
				(VariableNode "$a-parse")
			)
			(WordInstanceLink
				(VariableNode "$iobj")
				(VariableNode "$a-parse")
			)
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
