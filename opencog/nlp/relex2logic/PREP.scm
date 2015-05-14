(define PREP
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
				(VariableNode "$prep")
				(TypeNode "WordInstanceNode")
			)
			(TypedVariableLink
				(VariableNode "$obj")
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
					(VariableNode "$prep")
					(VariableNode "$a-parse")
				)
 				(WordInstanceLink
					(VariableNode "$obj")
					(VariableNode "$a-parse")
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_psubj")
					(ListLink
						(VariableNode "$prep")
						(VariableNode "$subj")
 					)
				)
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_pobj")
					(ListLink
						(VariableNode "$prep")
						(VariableNode "$obj")
 					)
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-prep-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$prep")
				(VariableNode "$obj")
			)
		)
	)
))


(InheritanceLink (stv 1 .99) (ConceptNode "PREP-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "PREP-Rule") PREP)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-prep-rule subj prep obj)
	(SVO-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str prep) (cog-name prep)
		(word-inst-get-word-str obj) (cog-name obj)
	)
)

