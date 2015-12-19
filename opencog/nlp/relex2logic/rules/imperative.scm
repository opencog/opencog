;
; Determine imperatives, based on LG link types.
;
(define imperative
	(BindLink
		(VariableList
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$wall-inst" "WordInstanceNode")
			(var-decl "$word-inst" "WordInstanceNode")
		)
		(AndLink
			(InterpretationLink
				(VariableNode "$interp")
				(VariableNode "$parse")
			)
			(word-in-parse "$wall-inst" "$parse")

			(ReferenceLink
				(VariableNode "$wall-inst")
				(WordNode "###LEFT-WALL###")
			)

			;; If left wall is linked with Wi,
			;; then its imperative.
			(EvaluationLink (LinkGrammarRelationshipNode "Wi")
				(ListLink
					(VariableNode "$wall-inst")
					(VariableNode "$word-inst")))
		)
		; Mark this as an imperative.
		(InheritanceLink
			(VariableNode "$interp")
			(DefinedLinguisticConceptNode "ImperativeSpeechAct"))
	)
)
