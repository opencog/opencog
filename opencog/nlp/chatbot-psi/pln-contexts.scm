;; PLN contexts

(use-modules (opencog logger))

;; TODO: put everything produced by PLN under a certain predicate, and
;; check if the current query has some common terms.
(define (is-pln-inferred-related?)
  (cog-logger-info "[PLN-Psi] is-pln-inferred-related?")
  (True)
)

(Define
    (DefinedPredicate "is-pln-inferred-related?")
    (is-pln-inferred-related?)
)

(Define
    (DefinedPredicate "pln-qa-not-started?")
    (search-not-started? pln-qa)
)

(Define
    (DefinedPredicate "pln-qa-finished?")
    (search-finished? pln-qa)
)

(Define
    (DefinedPredicate "is-pln-answer?")
    (any-result? pln-answers)
)
