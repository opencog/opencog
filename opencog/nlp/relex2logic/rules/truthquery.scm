;
; A truth-query detector, based on LG link types.
;
(define truthquery
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

			;; If left wall is linked with Qd,
			;; then it is a truth-query.
			(lg-link "Qd" "$wall-inst" "$word-inst")
		)
		; Mark this as a truth-query.
		(InheritanceLink
			(VariableNode "$interp")
			(DefinedLinguisticConceptNode "TruthQuerySpeechAct")
		)
	)
)
