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
                (EvaluationLink (LinkGrammarRelationshipNode "Wq")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Ws")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Wj")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Ww")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Qe")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))

                (EvaluationLink (LinkGrammarRelationshipNode "Qw")
                    (ListLink
                        (VariableNode "$wall-inst")
                        (VariableNode "$wh-word-inst")))
            )
        )
        ; Mark this as a question.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
    )
)
