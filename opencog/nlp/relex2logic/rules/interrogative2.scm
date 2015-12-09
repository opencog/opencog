;
; A more accurate interrogative detector, based on LG link types.
;
(define interrogative-LG
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$interp")
                (TypeNode "InterpretationNode")
            )
            (TypedVariableLink
                (VariableNode "$parse")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$wall-inst")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$wh-word-inst")
                (TypeNode "WordInstanceNode")
            )
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

            ;; If left wall is linked with any of Wq, Ws, Wj or Ww
            ;; or Qd, Qe, Qw then its interrogative.
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

                (EvaluationLink (LinkGrammarRelationshipNode "Qd")
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
