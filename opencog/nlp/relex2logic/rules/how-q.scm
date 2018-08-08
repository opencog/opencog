; This is the default "how" question, which is how-of-manner, as in "how did you sleep?"
; (AN June 2015)
;
; NB: in order to make this rule work, I have had to retain the classification of "how" as a "PrepositionalRelationshipNode" because that is
; what Relex is putting into the atomspace.  It makes no sense, but since I don't even know where is the code that generates the relex atoms,
; and changing it would probably muck up something else, so I'm just leaving it!

(define how-q
	(BindLink
		(VariableList
			(var-decl "$a-parse" "ParseNode")
			(var-decl "$verb" "WordInstanceNode")
			(var-decl "$qVar" "WordInstanceNode")
		)
		(AndLink
			(word-in-parse "$verb" "$a-parse")
			(word-in-parse "$qVar" "$a-parse")
			(EvaluationLink
				(PrepositionalRelationshipNode "how")
				(ListLink
					(VariableNode "$verb")
					(VariableNode "$qVar")
				)
 			)
		)
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: pre-how-q-rule")
			(ListLink
				(VariableNode "$verb")
			)
		)
	)
)

; This is function is not needed. It is added so as not to break the existing
; r2l pipeline.
(define-public (pre-how-q-rule verb)
	(how-rule (cog-name (word-inst-get-lemma verb)) (cog-name verb))
)
