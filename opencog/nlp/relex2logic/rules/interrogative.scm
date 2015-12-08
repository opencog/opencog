(define interrogative
    (BindLink
        (VariableList
            (TypedVariableLink
                (VariableNode "$word-inst-node")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$parse-node")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$interp-node")
                (TypeNode "InterpretationNode")
            )
        )
        (AndLink
            (WordInstanceLink
                (VariableNode "$word-inst-node")
                (VariableNode "$parse-node")
            )
; XXX FIXME This is a horrible strategy: link-grammar already
; does a more accurate job than relex for this stuff; LG uses
; the Wq and the Q links to identify questions!
;
; Done, with rule interrogative2 .. will keep this rule around
; for a while, as a backup plan.

            ; WH-words will be POS-tagged with "interrogative"
            (InheritanceLink
                (VariableNode "$word-inst-node")
                (DefinedLinguisticConceptNode "interrogative")
            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
        )
        ; mark this as a question
        (InheritanceLink
            (VariableNode "$interp-node")
            (DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
    )
)
