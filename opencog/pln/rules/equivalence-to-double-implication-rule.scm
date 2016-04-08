;; =============================================================================
;; EquivalenceToDoubleImplicationRule
;;
;; Equivalence
;;    A
;;    B
;; |-
;; Implication
;;    A
;;    B
;; Implication
;;    B
;;    A
;;
;; -----------------------------------------------------------------------------

(use-modules (opencog logger))

(define equivalence-to-double-implication-variables
  (VariableList
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-double-implication-forward-body
  (Equivalence
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-double-implication-backward-body-1
  (Implication
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-double-implication-backward-body-2
  (Implication
     (Variable "$B")
     (Variable "$A")))

(define equivalence-to-double-implication-rewrite
  (ExecutionOutput
     (GroundedSchema "scm: equivalence-to-double-implication-formula")
        (List
           equivalence-to-double-implication-backward-body-1
           equivalence-to-double-implication-backward-body-2
           equivalence-to-double-implication-forward-body)))

(define equivalence-to-double-implication-forward-rule
  (Bind
     equivalence-to-double-implication-variables
     equivalence-to-double-implication-forward-body
     equivalence-to-double-implication-rewrite))

(define equivalence-to-double-implication-backward-rule-1
  (Get
     equivalence-to-double-implication-variables
     equivalence-to-double-implication-backward-body-1))

(define equivalence-to-double-implication-backward-rule-2
  (Get
     equivalence-to-double-implication-variables
     equivalence-to-double-implication-backward-body-2))

(define equivalence-to-double-implication-rule
  (List
     equivalence-to-double-implication-forward-rule
     equivalence-to-double-implication-backward-rule-1
     equivalence-to-double-implication-backward-rule-2))

(define (equivalence-to-double-implication-formula AB BA EQ)
    (cog-set-tv!
       AB
       (equivalence-to-double-implication-side-effect-free-formula AB EQ))
    (cog-set-tv!
       BA
       (equivalence-to-double-implication-side-effect-free-formula BA EQ))
    (List AB BA))

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
  (DefinedSchema "equivalence-to-double-implication-rule"))
(Define
  equivalence-to-double-implication-rule-name
  equivalence-to-double-implication-rule)
