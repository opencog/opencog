;; 2. If and-BIT A is a proof of target T, then it is a preproof of
;;    target T as well.
;;
;; Implication <1 1>
;;   Predicate "URE:BC:proof"
;;   Predicate "URE:BC:preproof"
(define proof-is-preproof
  (Implication (stv 1 1)
    (Predicate "URE:BC:proof")
    (Predicate "URE:BC:preproof")))

;; Load conditional-full-instantiation-implication-meta-rule
(add-to-load-path "../../../opencog/pln/")
(load-from-path "meta-rules/predicate/conditional-full-instantiation.scm")

;; Turn proof-is-preproof into a rule
(define proof-is-preproof-rule
  (car (apply-rule conditional-full-instantiation-implication-meta-rule
                   proof-is-preproof)))
