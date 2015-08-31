; =============================================================================
; AndToContextRule
; 
; SubsetLink
;   AndLink
;       A
;       C
;   AndLink
;       B
;       C
; |-
; ContextLink
;   C
;   SubsetLink
;       A
;       B
;
; -----------------------------------------------------------------------------
(define pln-rule-and-to-context
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (SubsetLink
            (AndLink
                (VariableNode "$A")
                (VariableNode "$C"))
            (AndLink
                (VariableNode "$B")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm:pln-formula-and-to-context")
            (ListLink
                (SubsetLink
                    (AndLink
                        (VariableNode "$A")
                        (VariableNode "$C"))
                    (AndLink
                        (VariableNode "$B")
                        (VariableNode "$C")))
                (ContextLink
                    (VariableNode "$C")
                        (SubsetLink
                            (VariableNode "$A")
                            (VariableNode "$B")))))))

(define (pln-formual-and-to-context SACBC CAB)
    (cog-set-tv!
        CAB (cog-tv SACBC)))

		

; Name the rule
(define pln-rule-and-to-context-name (Node "pln-rule-and-to-context"))
(DefineLink pln-rule-and-to-context-name pln-rule-and-to-context)
