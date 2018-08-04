; TODO: Replace the following with pln's deduction-inheritance-rule when
; deduction-formula stops returning (cog-undefined-handle).
(define (gen-temp-deduction-rule link-type)
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (link-type
                (VariableNode "$A")
                (VariableNode "$B"))
            (link-type
                (VariableNode "$B")
                (VariableNode "$C"))
            (NotLink
                (EqualLink
                    (VariableNode "$A")
                    (VariableNode "$C")
                )))
        (link-type
            (VariableNode "$A")
            (VariableNode "$C")))
)

(define temp-deduction-inheritance-rule
    (gen-temp-deduction-rule InheritanceLink))
