;; =============================================================================
;; Fuzzy conjunction introduction rule
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
;; Where A1 to An are atoms with a fuzzy TV
;; -----------------------------------------------------------------------------

(use-modules (srfi srfi-1))

;; Generate variable (Variable prefix + "-" + to_string(i))
(define (gen-variable prefix i)
  (Variable (string-append prefix "-" (number->string i))))

;; Generate a list of variables (Variable prefix + "-" + to_string(n))
(define (gen-variables prefix n)
  (if (= n 0)
      ;; Base case
      '()
      ;; Recursive case
      (append (gen-variables prefix (- n 1))
              (list (gen-variable prefix (- n 1))))))

;; Generate a fuzzy conjunction introduction rule for an n-ary
;; conjunction
(define (gen-fuzzy-conjunction-introduction-rule nary)
  (let* ((variables (gen-variables "$X" nary))
         (EvaluationT (Type "EvaluationLink"))
         (InheritanceT (Type "InheritanceLink"))
         (OrT (Type "OrLink"))
         (NotT (Type "NotLink"))
         ;; Not AndLink because we'd rather have that already flattened
         (type (TypeChoice EvaluationT InheritanceT OrT NotT))
         (gen-typed-variable (lambda (x) (TypedVariable x type)))
         (vardecl (VariableList (map gen-typed-variable variables)))
         (pattern (And variables))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: fuzzy-conjunction-introduction-formula")
                    ;; We wrap the variables in Set because the order
                    ;; doesn't matter and this may speed up the URE.
                    (List (And variables) (Set variables)))))
    (Bind
      vardecl
      pattern
      rewrite)))

(define (fuzzy-conjunction-introduction-formula A S)
  (let* ((andees (cog-outgoing-set S))
         (min-s-atom (min-element-by-key andees cog-stv-strength))
         (min-c-atom (min-element-by-key andees cog-stv-confidence))
         (min-s (cog-stv-strength min-s-atom))
         (min-c (cog-stv-confidence min-c-atom)))
    (cog-merge-hi-conf-tv! A (stv min-s min-c))))

;; Name the rules
;;
;; Lame enumeration, maybe scheme can do better?
(define fuzzy-conjunction-introduction-1ary-rule-name
  (DefinedSchema "fuzzy-conjunction-introduction-1ary-rule"))
(DefineLink
  fuzzy-conjunction-introduction-1ary-rule-name
  (gen-fuzzy-conjunction-introduction-rule 1))
(define fuzzy-conjunction-introduction-2ary-rule-name
  (DefinedSchema "fuzzy-conjunction-introduction-2ary-rule"))
(DefineLink
  fuzzy-conjunction-introduction-2ary-rule-name
  (gen-fuzzy-conjunction-introduction-rule 2))
(define fuzzy-conjunction-introduction-3ary-rule-name
  (DefinedSchema "fuzzy-conjunction-introduction-3ary-rule"))
(DefineLink
  fuzzy-conjunction-introduction-3ary-rule-name
  (gen-fuzzy-conjunction-introduction-rule 3))
(define fuzzy-conjunction-introduction-4ary-rule-name
  (DefinedSchema "fuzzy-conjunction-introduction-4ary-rule"))
(DefineLink
  fuzzy-conjunction-introduction-4ary-rule-name
  (gen-fuzzy-conjunction-introduction-rule 4))
(define fuzzy-conjunction-introduction-5ary-rule-name
  (DefinedSchema "fuzzy-conjunction-introduction-5ary-rule"))
(DefineLink
  fuzzy-conjunction-introduction-5ary-rule-name
  (gen-fuzzy-conjunction-introduction-rule 5))
