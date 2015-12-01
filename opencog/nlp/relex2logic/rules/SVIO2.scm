; This rule is for snagging some of the indirect objects that aren't lucky enough to get
; the _iobj relation from relex.  So, instead theyre just object sof the preposition "to". Pooer things.
; as in "I sent the money to your boss."
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
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-svio-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$obj")
				(VariableNode "$iobj")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svio-rule subj verb obj iobj)
	(SVIO-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma  verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma  obj)) (cog-name obj)
		(cog-name (word-inst-get-lemma  iobj)) (cog-name iobj)

	)
)
