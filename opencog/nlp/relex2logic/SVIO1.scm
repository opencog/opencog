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
		)
		(ImplicationLink
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
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-svio1-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
				(VariableNode "$obj")
				(VariableNode "$iobj")
			)
		)
	)
))


(InheritanceLink (stv 1 .99) (ConceptNode "SVIO1-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "SVIO1-Rule") SVIO1)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svio-rule subj verb obj iobj)
	(SVIO-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str verb) (cog-name verb)
		(word-inst-get-word-str obj) (cog-name obj)
		(word-inst-get-word-str iobj) (cog-name iobj)

	)
)

