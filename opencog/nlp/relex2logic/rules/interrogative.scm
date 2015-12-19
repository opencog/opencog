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
			(dependency  "$wh-word-inst")) "$wall-inst")
        ; Mark this as a question.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
    )
)
