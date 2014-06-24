;; filter #2: anaphor and antecedent are both "feminine".
;; Example: She -> Nancy

(define filter-#2
    (AndLink
        (InheritanceLink
            (VariableNode "$word-inst-anaphor")
            (DefinedLinguisticConceptNode "feminine")
        )
        (InheritanceLink
            (VariableNode "$word-inst-antecedent")
            (DefinedLinguisticConceptNode "feminine")
        )
    )
)