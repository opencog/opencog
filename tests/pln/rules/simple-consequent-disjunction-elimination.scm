;; Simple KB to test consequent-disjunction-elimination-rule
(define P (Predicate "P" (stv 0.02 0.6)))
(define Q1 (Predicate "Q1" (stv 0.01 0.7)))
(define Q2 (Predicate "Q2" (stv 0.05 0.8)))

(Implication (stv 0.3 0.7)
   P
   (Or
     Q1
     Q2))

(Implication (stv 0.2 0.6)
   P
   Q2)
