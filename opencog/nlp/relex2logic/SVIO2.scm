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


(InheritanceLink (stv 1 .99) (ConceptNode "SVIO2-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "SVIO2-Rule") SVIO2)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svio-rule subj verb obj iobj)
	(SVIO-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str verb) (cog-name verb)
		(word-inst-get-word-str obj) (cog-name obj)
		(word-inst-get-word-str iobj) (cog-name iobj)

	)
)

