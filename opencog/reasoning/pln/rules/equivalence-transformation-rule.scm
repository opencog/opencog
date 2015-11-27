; =============================================================================
; EquivalenceTransformationRule
; 
; EquivalenceLink
;   A
;   B
; |-
; AndLink
;   ImplicationLink
;       A
;       B
;   ImplicationLink
;       B
;       A
;
; -----------------------------------------------------------------------------

(define equivalence-transformation-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (EquivalenceLink
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: equivalence-transformation-formula")
            (ListLink
                (AndLink
                    (ImplicationLink
                        (ConceptNode "$A")
                        (ConceptNode "$B"))
                    (ImplicationLink
                        (ConceptNode "$B")
                        (ConceptNode "$A")))
                (EquivalenceLink
                    (VariableNode "$A")
                    (VariableNode "$B"))))))

(define (equivalence-transformation-formula AII EV)
    (let (
            (IAB (gar AII))
            (IBA (gdr AII)))
         (cog-set-tv!
            IAB
            (equivalence-transformation-side-effect-free-formula IAB EV))
         (cog-set-tv!
            IBA
            (equivalence-transformation-side-effect-free-formula IBA EV))
    AII))

(define (equivalence-transformation-side-effect-free-formula IAB EV)
    (let* (
            (A (gar IAB))
            (B (gdr IAB))
            (sA (cog-stv-strength A))
            (sB (cog-stv-strength B))
            (sEV (cog-stv-strength EV))
            ; Formula based on PLN book formula for sim2inh
            (sIAB (/ (* (+ 1 (/ sB sA)) sEV) (+ 1 sEV))))
        (stv sIAB (cog-stv-confidence EV))))

; Name the rule
(define equivalence-transformation-rule-name
  (Node "equivalence-transformation-rule"))
(DefineLink
  equivalence-transformation-rule-name
  equivalence-transformation-rule)
