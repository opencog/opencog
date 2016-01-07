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

(use-modules (opencog logger))

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
               (ImplicationLink
                  (VariableNode "$A")
                  (VariableNode "$B"))
               (ImplicationLink
                  (VariableNode "$B")
                  (VariableNode "$A"))
               (EquivalenceLink
                  (VariableNode "$A")
                  (VariableNode "$B"))))))

(define (equivalence-to-double-implication-formula AB BA EQ)
    (cog-set-tv!
       AB
       (equivalence-to-double-implication-side-effect-free-formula AB EQ))
    (cog-set-tv!
       BA
       (equivalence-to-double-implication-side-effect-free-formula BA EQ))
    (ListLink AB BA))

(define (equivalence-to-double-implication-side-effect-free-formula AB EQ)
    (let* (
            (A (gar AB))
            (B (gdr AB))
            (sA (cog-stv-strength A))
            (sB (cog-stv-strength B))
            (sEQ (cog-stv-strength EQ))
            (cEQ (cog-stv-confidence EQ))
            (sAB (if (< 0.99 (* sEQ cEQ)) ; Hack to work around the
                                          ; last of distributional
                                          ; TV. If sEQ is high enough,
                                          ; we just set sAB as sEQ
                     sEQ
                     ;; Formula based on PLN book formula for sim2inh
                     (/ (* (+ 1 (/ sB sA)) sEQ) (+ 1 sEQ)))))
            (stv sAB cEQ)))

;; Name the rule
(define equivalence-to-double-implication-rule-name
  (DefinedSchemaNode "equivalence-to-double-implication-rule"))
(DefineLink
  equivalence-to-double-implication-rule-name
  equivalence-to-double-implication-rule)
