; =============================================================================
; And to Subset Rule1
;
; AndLink
;   AndLink
;       A
;       B
;   A
; |-
; SubsetLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define and-to-subset1-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (AndLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (VariableNode "$A"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: and-to-subset1-formula")
            (ListLink
                (AndLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (VariableNode "$A")
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

(define (and-to-subset1-formula AAB A SAB)
    (cog-set-tv! 
        SAB
        (if
            (= (cog-stv-strength A) 0)
            (stv 0 0)
            (stv 
                (/ (cog-stv-strength AAB) (cog-stv-strength A))
                (min (cog-stv-confidence AAB) (cog-stv-confidence A))))))
 
