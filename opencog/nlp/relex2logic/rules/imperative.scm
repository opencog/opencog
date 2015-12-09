;
; Determine imperatives, based on LG link types.
;
(define imperative
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

            ;; If left wall is linked with Wi,
            ;; then its imperative.
            (EvaluationLink (LinkGrammarRelationshipNode "Wi")
                (ListLink
                    (VariableNode "$wall-inst")
                    (VariableNode "$word-inst")))
        )
        ; Mark this as an imperative.
        (InheritanceLink
            (VariableNode "$interp")
            (DefinedLinguisticConceptNode "ImperativeSpeechAct"))
    )
)
