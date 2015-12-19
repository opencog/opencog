;
; A more accurate interrogative detector, based on LG link types.
;
(define interrogative
	(BindLink
		(VariableList
			(var-decl "$interp" "InterpretationNode")
			(var-decl "$parse" "ParseNode")
			(var-decl "$wall-inst" "WordInstanceNode")
			(var-decl "$wh-word-inst" "WordInstanceNode")
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

			;; If left wall is linked with any of Wq, Ws, Wj or Ww
			;; or Qe, Qw then its interrogative.
			(ChoiceLink
				(lg-link "Wq" "$wall-inst" "$wh-word-inst")
				(lg-link "Ws" "$wall-inst" "$wh-word-inst")
				(lg-link "Wj" "$wall-inst" "$wh-word-inst")
				(lg-link "Ww" "$wall-inst" "$wh-word-inst")
				(lg-link "Qe" "$wall-inst" "$wh-word-inst")
				(lg-link "Qw" "$wall-inst" "$wh-word-inst")
			)
		)
		; Mark this as a question.
		(InheritanceLink
			(VariableNode "$interp")
			(DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
	)
)
