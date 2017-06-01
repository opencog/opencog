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
(define and-to-context-rule
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
            (GroundedSchemaNode "scm: and-to-context-formula")
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

(define (and-to-context-formula SACBC CAB)
    (cog-set-tv!
        CAB (cog-tv SACBC)))

		

; Name the rule
(define and-to-context-rule-name (DefinedSchemaNode "and-to-context-rule"))
(DefineLink and-to-context-rule-name and-to-context-rule)
