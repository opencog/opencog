;; anaphor is "masculine"
;; antecedent is not "masculine"


(define filter-#1
    (AndLink
        (InheritanceLink
            (VariableNode "$word-inst-anaphor")
            (DefinedLinguisticConceptNode "masculine")
        )
        (NotLink
            (InheritanceLink
                (VariableNode "$word-inst-antecedent")
                (DefinedLinguisticConceptNode "masculine")
            )
        )
    )
)