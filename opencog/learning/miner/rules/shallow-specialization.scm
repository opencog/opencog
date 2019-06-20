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
;;   ...
;;   Evaluation (stv 1 1)
;;     Predicate "minsup"
;;     List
;;       <pattern-shallow-specialization-n>
;;       <texts>
;;       <ms>

(load "miner-rule-utils.scm")

;; Generate a shallow specialization rule. If unary is #t then it adds
;; the constraint that the pattern must be unary. This is to avoid
;; specializing conjunctions of abstract patterns that may result in
;; circumventing the the conjunction expansion rule heuristic.
;;
;; mv is the maximum number of variable allowed in the resulting
;; patterns.
(define (gen-shallow-specialization-rule unary mv)
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
      (Present minsup-pattern)
      (absolutely-true-eval minsup-pattern)
      (if unary (unary-conjunction-pattern-eval pattern) '()))
    (ExecutionOutput
      (GroundedSchema (string-append "scm: shallow-specialization-mv-"
                                     (number->string mv)
                                     "-formula"))
      (List
        (Set)               ; Cannot know the structure of the rule
                            ; conclusion in advance, because we don't
                            ; know the number of shallow specializations,
                            ; thus we cannot build the Set. Need to
                            ; support ConsLink, or ConsSetLink or
                            ; such. Or perhaps use Glob.
        minsup-pattern)))))

;; Shallow specialization formula
;;
;; mv is the maximimum number of variables
(define (gen-shallow-specialization-formula mv)
  (lambda (conclusion . premises)
    ;; (cog-logger-debug "gen-shallow-specialization-formula mv = ~a, conclusion = ~a, premises = ~a" mv conclusion premises)
    (if (= (length premises) 1)
        (let* ((minsup-pattern (car premises))
               (pattern (get-pattern minsup-pattern))
               (texts (get-texts minsup-pattern))
               (ms (get-ms minsup-pattern))
               (shaspes (cog-shallow-specialize pattern texts ms (Number mv)))
               (minsup-shaspe (lambda (x) (cog-set-tv!
                                           (minsup-eval x texts ms)
                                           (stv 1 1))))
               (minsup-shaspes (map minsup-shaspe (cog-outgoing-set shaspes))))
          (Set minsup-shaspes)))))

;; Instantiate shallow specialization formulae for different maximum
;; number of variables
(define shallow-specialization-mv-1-formula (gen-shallow-specialization-formula 1))
(define shallow-specialization-mv-2-formula (gen-shallow-specialization-formula 2))
(define shallow-specialization-mv-3-formula (gen-shallow-specialization-formula 3))
(define shallow-specialization-mv-4-formula (gen-shallow-specialization-formula 4))
(define shallow-specialization-mv-5-formula (gen-shallow-specialization-formula 5))
(define shallow-specialization-mv-6-formula (gen-shallow-specialization-formula 6))
(define shallow-specialization-mv-7-formula (gen-shallow-specialization-formula 7))
(define shallow-specialization-mv-8-formula (gen-shallow-specialization-formula 8))
(define shallow-specialization-mv-9-formula (gen-shallow-specialization-formula 9))
