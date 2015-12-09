;
; Determine declaratives, based on LG link types.
;
(define declarative
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
                (VariableNode "$word-inst")
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

            ;; If left wall is linked with any of Wd, Wt or Wa,
            ;; then its declarative.
            (ChoiceLink
                (EvaluationLink (LinkGrammarRelationshipNode "Wd")
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
