;; PLN contexts

(use-modules (srfi srfi-1))

(use-modules (opencog logger))

(load "pln-utils.scm")
(load "contexts.scm")

(define must-have-names '("people")) ;; hack: because the answer involves people

;; Check whether the query has common words with the inferred atoms
(define (is-pln-inferred-related?)
  ;; (cog-logger-info "[PLN-Context] is-pln-inferred-related?")

  ; TODO: Replace with what has been assked to be inferred upon.
  (let ((sentence-names (get-input-utterance-names)))
    (if (lset<= equal? must-have-names sentence-names)
        (stv 1 1)
        (stv 0 1)
    )
  )
)

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
