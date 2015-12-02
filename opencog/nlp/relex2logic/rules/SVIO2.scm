; This rule is for snagging some of the indirect objects that aren't
; lucky enough to get the _iobj relation from relex.  So, instead
; they're just object of the preposition "to". Poor things.
; Example: "I sent the money to your boss."
; (AN June 2015)

(define SVIO2
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
				(VariableNode "$to")
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
			(WordInstanceLink
				(VariableNode "$to")
				(VariableNode "$a-parse")
			)
			(LemmaLink
				(VariableNode "$to")
				(WordNode "to")
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
				(DefinedLinguisticRelationshipNode "_pobj")
				(ListLink
					(VariableNode "$to")
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
