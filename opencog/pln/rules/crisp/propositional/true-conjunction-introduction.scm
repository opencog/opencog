;; =============================================================================
;; Crisp conjunction introduction rule that only considers the
;; conjunction of true premises
;;
;; A1
;; ...
;; An
;; |-
;; AndLink
;;   A1
;;   ...
;;   An
;;
;; Where A1 to An are atoms with (stv 1 1)
;; -----------------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog ure))

;; Generate a crisp true conjunction introduction rule for an n-ary
;; conjunction
(define (gen-true-conjunction-introduction-rule nary)
  (let* ((variables (gen-variables "$X" nary))
         (EvaluationT (Type "EvaluationLink"))
         (InheritanceT (Type "InheritanceLink"))
         (OrT (Type "OrLink"))
         (NotT (Type "NotLink"))
         (ExecutionT (Type "ExecutionLink"))
         ;; Not AndLink because we'd rather have that already flattened
         (type (TypeChoice EvaluationT InheritanceT OrT NotT ExecutionT))
         (gen-typed-variable (lambda (x) (TypedVariable x type)))
         (vardecl (VariableList (map gen-typed-variable variables)))
         (gen-precondition (lambda (x) (absolutely-true-eval x)))
         (preconditions (map gen-precondition variables))
         (pattern (And (Present variables) preconditions))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: true-conjunction-introduction-formula")
                    ;; We wrap the variables in Set because the order
                    ;; doesn't matter and this may speed up the URE.
                    (List (And variables) (Set variables)))))
    (Bind
      vardecl
      pattern
      rewrite)))

(define (true-conjunction-introduction-formula A S)
  (let* ((andees (cog-outgoing-set S))
         (min-s-atom (min-element-by-key andees cog-mean))
         (min-c-atom (min-element-by-key andees cog-confidence))
         (min-s (cog-mean min-s-atom))
         (min-c (cog-confidence min-c-atom))
         (min-tv (stv min-s min-c)))
    (if (tv->bool min-tv)
        (cog-merge-hi-conf-tv! A min-tv))))

;; Name the rules
;;
;; Lame enumeration, maybe scheme can do better?
(define true-conjunction-introduction-1ary-rule-name
  (DefinedSchema "true-conjunction-introduction-1ary-rule"))
(DefineLink
  true-conjunction-introduction-1ary-rule-name
  (gen-true-conjunction-introduction-rule 1))
(define true-conjunction-introduction-2ary-rule-name
  (DefinedSchema "true-conjunction-introduction-2ary-rule"))
(DefineLink
  true-conjunction-introduction-2ary-rule-name
  (gen-true-conjunction-introduction-rule 2))
(define true-conjunction-introduction-3ary-rule-name
  (DefinedSchema "true-conjunction-introduction-3ary-rule"))
(DefineLink
  true-conjunction-introduction-3ary-rule-name
  (gen-true-conjunction-introduction-rule 3))
(define true-conjunction-introduction-4ary-rule-name
  (DefinedSchema "true-conjunction-introduction-4ary-rule"))
(DefineLink
  true-conjunction-introduction-4ary-rule-name
  (gen-true-conjunction-introduction-rule 4))
(define true-conjunction-introduction-5ary-rule-name
  (DefinedSchema "true-conjunction-introduction-5ary-rule"))
(DefineLink
  true-conjunction-introduction-5ary-rule-name
  (gen-true-conjunction-introduction-rule 5))
