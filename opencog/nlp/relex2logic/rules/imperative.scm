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
			(interp-of-parse "$interp" "$parse")
			(word-in-parse "$wall-inst" "$parse")

			; Comma: for example "Eva, look up" links Wi to comma.
			(ChoiceLink
				(ReferenceLink
					(VariableNode "$wall-inst")
					(WordNode "###LEFT-WALL###"))
				(ReferenceLink
					(VariableNode "$wall-inst")
					(WordNode ","))
			)

			;; If left wall (or a comma) is linked with Wi,
			;; then its imperative.
			(lg-link "Wi" "$wall-inst" "$word-inst")
		)
		; Mark this as an imperative.
		(InheritanceLink
			(VariableNode "$interp")
			(DefinedLinguisticConceptNode "ImperativeSpeechAct"))
	)
)
