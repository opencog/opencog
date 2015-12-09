;
; A truth-query detector, based on LG link types.
;
(define truthquery
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

            ;; If left wall is linked with Qd,
            ;; then it is a truth-query.
            (EvaluationLink (LinkGrammarRelationshipNode "Qd")
                (ListLink
                    (VariableNode "$wall-inst")
                    (VariableNode "$word-inst")))
        )
        ; Mark this as a truth-query.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "TruthQuerySpeechAct"))
    )
)
