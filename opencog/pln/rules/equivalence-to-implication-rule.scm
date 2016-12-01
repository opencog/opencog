;; =============================================================================
;; EquivalenceToImplicationRule
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
;; In practice we only need to provide the rewrite term for
;;
;; Implication
;;    A
;;    B
;;
;; as B->A will automatically be generated due to EquivalenceLink
;; being symmetric.
;; -----------------------------------------------------------------------------

(use-modules (opencog logger))

(define equivalence-to-implication-vardecl
  (VariableList
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-implication-body
  (Equivalence
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-implication-output
  (Implication
     (Variable "$A")
     (Variable "$B")))

(define equivalence-to-implication-rewrite
  (ExecutionOutput
     (GroundedSchema "scm: equivalence-to-implication-formula")
        (List
           equivalence-to-implication-body
           equivalence-to-implication-output)))

(define equivalence-to-implication-rule
  (Bind
     equivalence-to-implication-vardecl
     equivalence-to-implication-body
     equivalence-to-implication-rewrite))

(define (equivalence-to-implication-formula EQ AB)
  (cog-set-tv!
     AB
     (equivalence-to-implication-side-effect-free-formula EQ AB)))

(define (equivalence-to-implication-side-effect-free-formula EQ AB)
    (let* (
            (A (gar AB))
            (B (gdr AB))
            (sA (cog-stv-strength A))
            (sB (cog-stv-strength B))
            (sEQ (cog-stv-strength EQ))
            (cEQ (cog-stv-confidence EQ))
            (sAB (if (< 0.99 (* sEQ cEQ)) ; Hack to work around the
                                          ; lack of distributional
                                          ; TV. If sEQ is high enough,
                                          ; we just set sAB as sEQ
                     sEQ
                     ;; Formula based on PLN book formula for sim2inh
                     (/ (* (+ 1 (/ sB sA)) sEQ) (+ 1 sEQ)))))
            (stv sAB cEQ)))

;; Name the rule
(define equivalence-to-implication-rule-name
  (DefinedSchema "equivalence-to-implication-rule"))
(Define
  equivalence-to-implication-rule-name
  equivalence-to-implication-rule)
