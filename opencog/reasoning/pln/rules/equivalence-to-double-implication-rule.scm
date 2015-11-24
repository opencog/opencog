;; =============================================================================
;; EquivalenceToDoubleImplicationRule
;;
;; EquivalenceLink
;;    A
;;    B
;; |-
;; ImplicationLink
;;    A
;;    B
;; ImplicationLink
;;    B
;;    A
;;
;; -----------------------------------------------------------------------------

(define equivalence-to-double-implication-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B"))
        (EquivalenceLink
            (VariableNode "$A")
            (VariableNode "$B"))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: equivalence-to-double-implication-formula")
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

(define (equivalence-to-double-implication-formula AII EV)
    (let (
            (IAB (gar AII))
            (IBA (gdr AII)))
         (cog-set-tv!
            IAB
            (equivalence-to-double-implication-side-effect-free-formula IAB EV))
         (cog-set-tv!
            IBA
            (equivalence-to-double-implication-side-effect-free-formula IBA EV))
    AII))

(define (equivalence-to-double-implication-side-effect-free-formula IAB EV)
    (let* (
            (A (gar IAB))
            (B (gdr IAB))
            (sA (cog-stv-strength A))
            (sB (cog-stv-strength B))
            (sEV (cog-stv-strength EV))
            ;; Formula based on PLN book formula for sim2inh
            (sIAB (/ (* (+ 1 (/ sB sA)) sEV) (+ 1 sEV))))
        (stv sIAB (cog-stv-confidence EV))))

;; Name the rule
(define equivalence-to-double-implication-rule-name
  (Node "equivalence-to-double-implication-rule"))
(DefineLink
  equivalence-to-double-implication-rule-name
  equivalence-to-double-implication-rule)
