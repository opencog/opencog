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
            (WordInstanceLink
                (VariableNode "$wall-inst")
                (VariableNode "$parse")
            )

            (ReferenceLink
                (VariableNode "$wall-inst")
                (WordNode "###LEFT-WALL###")
            )

            ;; If left wall is linked with any of Wd, Wp, Wr, Wt or Wa,
            ;; then its declarative.
            (ChoiceLink
                (EvaluationLink (LinkGrammarRelationshipNode "Wd")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Wp")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Wr")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Wt")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Wa")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$word-inst")))
            )
        )

        ; Mark this as declarative.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))
    )
)
