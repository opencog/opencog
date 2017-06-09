;; =============================================================================
;; Fuzzy negation introduction rule
;;
;; A
;; |-
;; NotLink
;;   A
;; -----------------------------------------------------------------------------

(use-modules (srfi srfi-1))

;; Negation rule
(define negation-introduction-rule
  (let* ((X (Variable "$X"))
         (EvaluationT (Type "EvaluationLink"))
         (InheritanceT (Type "InheritanceLink"))
         (PredicateT (Type "PredicateNode"))
         (ConceptT (Type "ConceptNode"))
         (AndT (Type "AndLink"))
         (OrT (Type "OrLink"))
         ;; Not NotLink because we'd rather have that already flattened
         (type (TypeChoice EvaluationT InheritanceT PredicateT ConceptT))
         (gen-typed-variable (lambda (x) (TypedVariable x type)))
         (vardecl (TypedVariable X type))
         (pattern (And
                    X
                    ;; precondition
                    (Evaluation
                      (GroundedPredicate "scm: gt-zero-confidence")
                      X)))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: negation-introduction-formula")
                    ;; We wrap the variables in Set because the order
                    ;; doesn't matter and this may speed up the URE.
                    (List (Not X) X))))
    (Bind
      vardecl
      pattern
      rewrite)))

(define (negation-introduction-formula N A)
  (let* ((A-s (cog-stv-strength A))
         (A-c (cog-stv-confidence A)))
    (cog-merge-hi-conf-tv! N (stv (- 1 A-s) A-c))))

;; Name the rules
(define negation-introduction-rule-name
  (DefinedSchema "negation-introduction-rule"))
(DefineLink
  negation-introduction-rule-name
  negation-introduction-rule)
