;; PLN inference carried over the background knowledge to overwrite
;; the truth value of the MOSES model based solely on the dataset.
(use-modules (opencog) (opencog exec))

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-fc-config.scm")

;; Apply the inference rules using the pattern matcher. This only
;; contains the executions, see the README.md for the explanations and
;; the expected results.

;; (1)
(for-each (lambda (i) (cog-execute! implication-partial-instantiation-rule))
          (iota 2))

;; (2)
(cog-execute! implication-scope-to-implication-rule)

;; (3)
(cog-execute! and-lambda-distribution-rule)

;; (4)
(cog-execute! closed-lambda-evaluation-rule)

;; (5)
(cog-execute! implication-introduction-rule)

;; (6)
(cog-execute! implication-implicant-distribution-rule)

;; (7)
(cog-execute! implication-and-lambda-factorization-rule)

;; (8)
(cog-execute! deduction-implication-rule)

;; (9)
(cog-execute! deduction-implication-rule)

;; (10)
(cog-execute! implication-full-instantiation-rule)

;; (11)
(cog-execute! equivalence-to-implication-rule)

;; (12)
(cog-execute! deduction-implication-rule)

;; (13)
(cog-execute! deduction-implication-rule)

;; (14)
(cog-execute! deduction-implication-rule)

;; (15)
;; (cog-execute! deduction-implication-rule)

;; (16)
(cog-execute! implication-implicant-disjunction-rule)
