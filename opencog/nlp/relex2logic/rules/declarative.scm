(define declarative
    (BindLink
        (VariableList
;            (TypedVariableLink
;                (VariableNode "$word-inst-node")
;                (TypeNode "WordInstanceNode")
;            )
            (TypedVariableLink
                (VariableNode "$parse-node")
                (TypeNode "ParseNode")
            )
            (TypedVariableLink
                (VariableNode "$interp-node")
                (TypeNode "InterpretationNode")
            )
            (TypedVariableLink
                (VariableNode "$word-inst-node-1")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word-inst-node-2")
                (TypeNode "WordInstanceNode")
            )
            (TypedVariableLink
                (VariableNode "$word-inst-node-3")
                (TypeNode "WordInstanceNode")
            )
        )
        (AndLink
;
; Looking for a period at the end of a sentence is far too
; strong a requirement -- most people on IRC chat will not
; be punctuating correctly, and certainly, speech-to-text will
; not be generating correctly-punctuated sentences.
;
;            (LemmaLink
;                (VariableNode "$word-inst-node")
;                (WordNode ".")
;            )
;            (PartOfSpeechLink
;                (VariableNode "$word-inst-node")
;                (DefinedLinguisticConceptNode "punctuation")
;            )
;            (WordInstanceLink
;                (VariableNode "$word-inst-node")
;                (VariableNode "$parse-node")
;            )
            (InterpretationLink
                (VariableNode "$interp-node")
                (VariableNode "$parse-node")
            )
            (AbsentLink
                (InheritanceLink
                    (VariableNode "$word-inst-node-1")
                    (DefinedLinguisticConceptNode "imperative")
                )
            )
            (AbsentLink
                (InheritanceLink
                    (VariableNode "$word-inst-node-2")
                    (DefinedLinguisticConceptNode "interrogative")
                )
            )
            (AbsentLink
                (InheritanceLink
                    (VariableNode "$word-inst-node-3")
                    (DefinedLinguisticConceptNode "truth-query")
                )
            )
        )
        ; mark this as declarative
        (InheritanceLink
            (VariableNode "$interp-node")
            (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))
    )
)
