;; PLN inference carried over the background knowledge to overwrite
;; the truth value of the MOSES model based solely on the dataset.

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

;; Apply the inference rules using the pattern matcher. This only
;; contains the executions, see the README.md for the explanations and
;; the expected results.

;; (1)
(for-each (lambda (i) (cog-bind implication-partial-instantiation-rule))
          (iota 2))

;; (2)
(cog-bind implication-lambda-distribution-rule)

;; (3)
(cog-bind and-lambda-distribution-rule)

;; (4)
(cog-bind lambda-grounded-construction-rule)

;; (5)
(cog-bind implication-construction-rule)

;; (6)
(cog-bind implication-implicant-distribution-rule)

;; (7)
(cog-bind implication-and-lambda-factorization-rule)

;; (8)
(cog-bind deduction-implication-rule)

;; (9)
;;
;; Actually the previous deduction step took care of that too, so no
;; need to run it
;;
;; (cog-bind deduction-implication-rule)

;; (10)
(cog-bind implication-full-instantiation-rule)

;; (11)
(cog-bind (ure-get-forward-rule equivalence-to-double-implication-rule))

;; (12)
(cog-bind deduction-implication-rule)

;; (13)
(cog-bind deduction-implication-rule)

;; (14)
(cog-bind deduction-implication-rule)

;; (15)
;;
;; Actually the previous deduction step took care of that too, so no
;; need to run it
;;
;; (cog-bind deduction-implication-rule)

;; (16)
(cog-bind implication-implicant-disjunction-rule)
