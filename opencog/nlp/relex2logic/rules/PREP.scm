; This rule is for sentences in which the main predicate is basically a preposition,
; as in "The book is ON the table." "You are OVER the top."  This rule only assigns
; the preposition to the subject, the object of the preposition is assigned by another
; rule.
; (AN June 2015)


(define PREP
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$subj" "WordInstanceNode")
			(var-decl "$prep" "WordInstanceNode")
			(var-decl "$obj" "WordInstanceNode")
		)
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
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define (pre-prep-rule subj prep obj)
	(SVO-rule (cog-name (word-inst-get-lemma  subj)) (cog-name subj)
		(cog-name (word-inst-get-lemma  prep)) (cog-name prep)
		(cog-name (word-inst-get-lemma obj)) (cog-name obj)
	)
)
