;; Define 2 dummy predicates

(PredicateNode "P" (stv 0.2 1))
(PredicateNode "Q" (stv 0.6 1))

;; ;; Define a more complex predicate with Lambda

;; (LambdaLink (stv 0.2 1)
;;    (VariableNode "$X")
;;    (EvaluationLink
;;       (PredicateNode "P")
;;       (VariableNode "$X")))

;; (LambdaLink (stv 0.6 1)
;;    (VariableNode "$X")
;;    (EvaluationLink
;;       (PredicateNode "Q")
;;       (VariableNode "$X")))
