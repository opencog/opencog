(define SV
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
				(EvaluationLink
					(DefinedLinguisticRelationshipNode "_subj")
					(ListLink
						(VariableNode "$verb")
						(VariableNode "$subj")
 					)
				)
				(AbsentLink
					(DefinedLinguisticRelationshipNode "_obj")
				)				
				(AbsentLink
					(DefinedLinguisticRelationshipNode "_pobj")
				)
				(AbsentLink
					(DefinedLinguisticRelationshipNode "_to-be")
				)
				(AbsentLink
					(DefinedLinguisticRelationshipNode "_predadj")
				)
			)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-sv-rule")
			(ListLink
				(VariableNode "$subj")
				(VariableNode "$verb")
			)
		)
	)
))


(InheritanceLink (stv 1 .99) (ConceptNode "SV-Rule") (ConceptNode "Rule"))

(ReferenceLink (stv 1 .99) (ConceptNode "SV-Rule") SV)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-sv-rule subj verb)
	(SV-rule (word-inst-get-word-str subj) (cog-name subj)
		(word-inst-get-word-str verb) (cog-name verb)
	)
)

