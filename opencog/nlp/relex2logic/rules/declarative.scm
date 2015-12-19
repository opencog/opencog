;
; Determine declaratives, based on LG link types.
;
(define declarative
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

            ;; If left wall is linked with any of Wd, Wp, Wr, Wt or Wa,
            ;; then its declarative.
            (ChoiceLink
			(dependency  "$word-inst")) "$wall-inst")

        ; Mark this as declarative.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))
    )
)
