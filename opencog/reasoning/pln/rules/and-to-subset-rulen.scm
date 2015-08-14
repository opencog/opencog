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
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define pln-rule-and-to-subsetn
    (BindLink
        (VariableList
            (TypedVariableNode
                (VariableNode "$A")
                (TypeNode "AndLink"))
            (TypedVariableNode
                (VariableNode "$B")
                (TypeNode "AndLink")))
        (AndLink
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: pln-formula-and-to-subsetn")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")))))

(define (pln-formula-and-to-subsetn ABCD ABC)
    (cond
        [(eq? 
            (cog-outgoing-set ABC) 
            (reverse (cdr (reverse (cog-outgoing-set ABCD)))))
         (cog-set-tv!
            (SubsetLink
                ABC
                (first (reverse (cog-outgoing-set ABCD))))
            (stv 
                (/ (cog-stv-strength ABCD) (cog-stv-strength ABC)) 
                (min (cog-stv-confidence ABCD) (cog-stv-confidence ABC))))]))

