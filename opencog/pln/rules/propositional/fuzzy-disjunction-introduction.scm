;; =============================================================================
;; Fuzzy disjunction introduction rule
;;
;; A1
;; ...
;; An
;; |-
;; OrLink
;;   A1
;;   ...
;;   An
;;
;; Where A1 to An are atoms with a fuzzy TV
;; -----------------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog ure))

;; Generate a fuzzy disjunction introduction rule for an n-ary
;; disjunction
(define (gen-fuzzy-disjunction-introduction-rule nary)
  (let* ((variables (gen-variables "$X" nary))
         (EvaluationT (Type "EvaluationLink"))
         (InheritanceT (Type "InheritanceLink"))
         (AndT (Type "AndLink"))
         (NotT (Type "NotLink"))
         ;; Not OrLink because we'd rather have that already flattened
         (type (TypeChoice EvaluationT InheritanceT AndT NotT))
         (gen-typed-variable (lambda (x) (TypedVariable x type)))
         (vardecl (VariableList (map gen-typed-variable variables)))
         (pattern (And variables))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: fuzzy-disjunction-introduction-formula")
                    ;; We wrap the variables in Set because the order
                    ;; doesn't matter and this may speed up the URE.
                    (List (Or variables) (Set variables)))))
    (Bind
      vardecl
      pattern
      rewrite)))

(define (fuzzy-disjunction-introduction-formula A S)
  (let* ((orees (cog-outgoing-set S))
         (max-s-atom (max-element-by-key orees cog-mean))
         (min-c-atom (min-element-by-key orees cog-confidence))
         (max-s (cog-mean max-s-atom))
         (min-c (cog-confidence min-c-atom)))
    (cog-merge-hi-conf-tv! A (stv max-s min-c))))

;; Name the rules
;;
;; Lame enumeration, maybe scheme can do better?
(define fuzzy-disjunction-introduction-1ary-rule-name
  (DefinedSchema "fuzzy-disjunction-introduction-1ary-rule"))
(DefineLink
  fuzzy-disjunction-introduction-1ary-rule-name
  (gen-fuzzy-disjunction-introduction-rule 1))
(define fuzzy-disjunction-introduction-2ary-rule-name
  (DefinedSchema "fuzzy-disjunction-introduction-2ary-rule"))
(DefineLink
  fuzzy-disjunction-introduction-2ary-rule-name
  (gen-fuzzy-disjunction-introduction-rule 2))
(define fuzzy-disjunction-introduction-3ary-rule-name
  (DefinedSchema "fuzzy-disjunction-introduction-3ary-rule"))
(DefineLink
  fuzzy-disjunction-introduction-3ary-rule-name
  (gen-fuzzy-disjunction-introduction-rule 3))
(define fuzzy-disjunction-introduction-4ary-rule-name
  (DefinedSchema "fuzzy-disjunction-introduction-4ary-rule"))
(DefineLink
  fuzzy-disjunction-introduction-4ary-rule-name
  (gen-fuzzy-disjunction-introduction-rule 4))
(define fuzzy-disjunction-introduction-5ary-rule-name
  (DefinedSchema "fuzzy-disjunction-introduction-5ary-rule"))
(DefineLink
  fuzzy-disjunction-introduction-5ary-rule-name
  (gen-fuzzy-disjunction-introduction-rule 5))
