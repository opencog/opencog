;; filter #4: anaphor is "neuter", antecedent is not "feminine" or "masculine"
;;            anaphor and antecedent are both "singular"
;; Example: it -> a house

(define filter-#4
    (ListLink
        (AnchorNode "CurrentPronoun")
        (VariableNode "$word-inst-anaphor")
    )
)
