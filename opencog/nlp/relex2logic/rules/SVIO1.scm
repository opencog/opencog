; This rule is for sentences in which the indirect object is detected properly by relex,
; such as "I gave the dog some LSD."
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
		)
   (ListLink
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
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-svio-rule subj verb obj iobj)
 (ListLink
	(SVIO-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma  verb)) (cog-name verb)
		(cog-name (word-inst-get-lemma  obj)) (cog-name obj)
		(cog-name (word-inst-get-lemma  iobj)) (cog-name iobj)

	)
 )
)
