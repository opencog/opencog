; =============================================================================
; Inheritance Rule
;
; AndLink
;   SubsetLink
;       A
;       B
;   IntensionalInheritanceLink
;       A
;       B
; |-
; InheritanceLink
;   A
;   B
;
; -----------------------------------------------------------------------------

(define inheritance-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (IntensionalInheritanceLink
                (VariableNode "$A")
                (VariableNode "$B")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: inheritance-formula")
            (ListLink
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (IntensionalInheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

(define (inheritance-formula SAB IIAB IAB)
    (cog-set-tv! 
        IAB
        (stv 
            (/ 
                (+ (cog-mean SAB) (cog-mean IIAB)) 
                2.0) 
            (min (cog-confidence SAB) (cog-confidence IIAB)))))
 
