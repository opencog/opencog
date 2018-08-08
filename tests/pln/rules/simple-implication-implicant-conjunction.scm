;; Simple KB to test implication-implicant-conjunction-rule
(define P1 (Predicate "P1" (stv 0.02 0.6)))
(define P2 (Predicate "P2" (stv 0.01 0.7)))
(define Q (Predicate "Q" (stv 0.05 0.8)))

(Implication (stv 0.1 0.7)
   P1
   Q)

(Implication (stv 0.2 0.6)
   P2
   Q)
