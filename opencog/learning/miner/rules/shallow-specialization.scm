;; Rule combine both a shallow abstraction and a specialization in one
;; step.
;;
;; The advantage of that rule is that it doesn't create intermediary
;; abstraction structure such as
;;
;; Evaluation
;;   Predicate "abstraction"
;;   List
;;     <abstractions>
;;     minsup(pattern, texts, ms)
;;
;; Instead, given a minsup(pattern, texts, ms) evaluation, it directly
;; generate shallow specializations of pattern.
;;
;; Evaluation (stv 1 1)
;;   Predicate "minsup"
;;   List
;;     <pattern>
;;     <texts>
;;     <ms>
;; |-
;; Set
;;   Evaluation (stv 1 1)
;;     Predicate "minsup"
;;     List
;;       <pattern-shallow-specialization-1>
;;       <texts>
;;       <ms>
;; ...

(load "miner-rule-utils.scm")

;; Generate a shallow specialization rule. If unary is #t then it adds
;; the constraint that the pattern must be unary. This is to avoid
;; specializing conjunctions of abstract patterns that may result in
;; circumventing the the conjunction expansion rule heuristic.
(define (gen-shallow-specialization-rule unary)
  (let* (;; Variables
         (pattern (Variable "$pattern"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         ;; Types
         (LambdaT (Type "LambdaLink"))
         (ConceptT (Type "ConceptNode"))
         (NumberT (Type "NumberNode"))
         ;; Vardecls
         (pattern-decl (TypedVariable pattern LambdaT))
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         ;; Clauses
         (minsup-pattern (minsup-eval pattern texts ms)))
  (Bind
    (VariableList
      pattern-decl
      texts-decl
      ms-decl)
    (And
      minsup-pattern
      (absolutely-true-eval minsup-pattern)
      (if unary (unary-conjunction-pattern-eval pattern) '()))
    (ExecutionOutput
      (GroundedSchema "scm: shallow-specialization-formula")
      (List
        (Set)               ; Cannot know the structure of the rule
                            ; conclusion in advance, because we don't
                            ; know the number of shallow specializations,
                            ; thus we cannot build the Set. Need to
                            ; support ConsLink, or ConsSetLink or
                            ; such. Or perhaps use Glob.
        minsup-pattern)))))

;; Shallow specialization formula
(define (shallow-specialization-formula conclusion . premises)
  ;; (cog-logger-debug "shallow-specialization-formula conclusion = ~a, premises = ~a" conclusion premises)
  (if (= (length premises) 1)
      (let* ((minsup-pattern (car premises))
             (pattern (get-pattern minsup-pattern))
             (texts (get-texts minsup-pattern))
             (ms (get-ms minsup-pattern))
             (shaspes (cog-shallow-specialize pattern texts ms))
             (minsup-shaspe (lambda (x) (cog-set-tv!
                                         (minsup-eval x texts ms)
                                         (stv 1 1))))
             (minsup-shaspes (map minsup-shaspe (cog-outgoing-set shaspes))))
        (Set minsup-shaspes))))

;; Define shallow specialization
(define shallow-specialization-rule-name
  (DefinedSchemaNode "shallow-specialization-rule"))
(DefineLink shallow-specialization-rule-name
  (gen-shallow-specialization-rule #f))

;; Define shallow specialization for unary patterns
(define shallow-specialization-unary-rule-name
  (DefinedSchemaNode "shallow-specialization-unary-rule"))
(DefineLink shallow-specialization-unary-rule-name
  (gen-shallow-specialization-rule #t))
