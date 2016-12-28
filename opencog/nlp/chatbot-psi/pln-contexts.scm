;; PLN contexts

(use-modules (srfi srfi-1))

(use-modules (opencog logger))
(use-modules (opencog query))

(load "pln-utils.scm")

;; Check whether the query has common words with the inferred atoms
(define (is-pln-inferred-related?)
  ;; (cog-logger-debug "[PLN-Context] is-pln-inferred-related?")

  (let ((must-have-names '("people")) ;; hack: because the answer involves people
        (sentence-names (get-input-utterance-names)))
    (if (lset<= equal? must-have-names sentence-names)
        (let* (
               (inferred-names (get-inferred-names))
               (inter-names (lset-intersection equal?
                                               inferred-names
                                               sentence-names)))
          ;; (cog-logger-debug "[PLN-Context] inferred-names = ~a" inferred-names)
          ;; (cog-logger-debug "[PLN-Context] sentence-names = ~a" sentence-names)
          ;; (cog-logger-debug "[PLN-Context] inter-names = ~a" inter-names)
          (if (null? inter-names)
              (stv 0 1)
              (stv 1 1)))
        (stv 0 1))))

(Define
    (DefinedPredicate "is-pln-inferred-related?")
    (Evaluation
       (GroundedPredicate "scm: is-pln-inferred-related?")
       (List))
)

(Define
    (DefinedPredicate "pln-qa-not-started?")
    (process-not-started? pln-qa)
)

(Define
    (DefinedPredicate "pln-qa-finished?")
    (process-finished? pln-qa)
)

(Define
    (DefinedPredicate "is-pln-answer?")
    (any-result? pln-answers)
)
