; =============================================================================
; And To Subset Rule N
;
; AndLink
;   AndLink
;       A
;       B
;       C
;       D
;   AndLink
;       A
;       B
;       C
; |-
; SubsetLink
;   AndLink
;       A
;       B
;       C
;   D
;
; To solve the pattern matcher issue, and-to-subsetn rule has been divided into
; two parts. The two rules are :-
;       and-to-subset-3-rule
;       and-to-subset-4-rule
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define and-to-subset-3-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C"))
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: and-to-subsetn-formula")
            (ListLink
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$B"))
                    (VariableNode "$C"))))))

(define and-to-subset-4-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C")
            (VariableNode "$D"))
        (AndLink
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")
                (VariableNode "$D"))
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: and-to-subsetn-formula")
            (ListLink
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                    (VariableNode "$C")
                    (VariableNode "$D"))
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B")
                    (VariableNode "$C"))
                (SubsetLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$B")
                        (VariableNode "$C"))
                    (VariableNode "$D"))))))

(define (and-to-subsetn-formula ABCD ABC sABCD)
    (cog-set-tv!
        sABCD
        (stv 
            (/ (cog-stv-strength ABCD) (cog-stv-strength ABC)) 
            (min (cog-stv-confidence ABCD) (cog-stv-confidence ABC)))))

