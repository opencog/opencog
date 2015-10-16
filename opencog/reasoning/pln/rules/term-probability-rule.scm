; =============================================================================
; Term Probability Rule
; 
; AndLink
;   LinkType
;       A
;       B
;   LinkType
;       B
;       A
; A
; |-
; B
;
; Due to issues in pattern matching in backward chaining, the files has been
; split into three rules for seperate link types. The 3 rules are
;           term-probability-inheritance-rule
;           term-probability-implication-rule
;           term-probability-subset-rule
;
; -----------------------------------------------------------------------------
(load "formulas.scm")

(define term-probability-inheritance-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (InheritanceLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (InheritanceLink
                (VariableNode "$B")
                (VariableNode "$A")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: term-probability-formula")
            (ListLink
                (VariableNode "$A")
                (InheritanceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (InheritanceLink
                    (VariableNode "$B")
                    (VariableNode "$A"))
                (VariableNode "$B")))))

(define term-probability-implication-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (VariableNode "$B")
                (VariableNode "$A")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: term-probability-formula")
            (ListLink
                (VariableNode "$A")
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$B")
                    (VariableNode "$A"))
                (VariableNode "$B")))))

(define term-probability-subset-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (AndLink
            (VariableNode "$A")
            (SubsetLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (SubsetLink
                (VariableNode "$B")
                (VariableNode "$A")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: term-probability-formula")
            (ListLink
                (VariableNode "$A")
                (SubsetLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (SubsetLink
                    (VariableNode "$B")
                    (VariableNode "$A"))
                (VariableNode "$B")))))

(define (term-probability-formula A AB BA B)
    (let
        ((sA (cog-stv-strength A))
         (cA (cog-stv-confidence A))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBA (cog-stv-strength BA))
         (cBA (cog-stv-confidence BA)))
        (cog-set-tv! 
            B 
            (stv 
                (/ (* sA sAB) sBA) 
                (min (min cAB cBA) cA)))))
