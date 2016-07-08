;; PLN actions

;; TODO: call a fuzzy matcher (perhaps the default fuzzy matcher) on the
;; set of PLN inferred knowledge and pick up the one that matches the
;; most with the query
(define-public (do-pln-QA)
    (State pln-qa search-started)

    (cog-logger-info "[PLN-Psi] do-pln-QA")

    (State pln-answers (List (Word "I") (Word "am") (Word "the") (Word "PLN") (Word "answer")))

    (State pln-qa search-finished)
)
