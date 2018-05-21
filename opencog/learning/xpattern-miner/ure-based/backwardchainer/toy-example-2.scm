;; Let be a binary predicate R and a unary predicate Q. The pattern we
;; want to discover is
;;
;; if R(X, Inheritance("a", Y)) then Q(X)
;;
;; starting from the abstraction
;;
;; if R(X, Y) then Q(X)
;;
;; Given a number of instances of R and Q
;;
;; That is, in atomese, the representation of the abstraction is
;;
;; ImplicationScope
;;   VariableList
;;     Variable "$X"
;;     Variable "$Y"
;;   Evaluation
;;     Predicate "R"
;;     List
;;       Variable "$X"
;;       Variable "$Y"
;;   Evaluation
;;     Predicate "Q"
;;     Variable "$X"
;;
;; and the representation of the specialization is
;;
;; ImplicationScope <rule-TV>
;;   VariableList
;;     Variable "$X"
;;     Variable "$Y"
;;   Evaluation
;;     Predicate "R"
;;     List
;;       Variable "$X"
;;       Inheritance
;;         Concept "a"
;;         Variable "$Y"
;;   Evaluation
;;     Predicate "Q"
;;     Variable "$X"
;;
;; For that we build the backward chainer query
;;
;; Compose
;;   ImplicationScope <rule-TV>
;;     VariableList
;;       Variable "$X"
;;       Variable "$Y"
;;     Evaluation
;;       Predicate "R"
;;       List
;;         Variable "$X"
;;         Variable "$Y"
;;     Evaluation
;;       Predicate "Q"
;;       Variable "$X"
;;   List
;;     Project 0
;;     Variable "$Y-specialization"
;;
;; where "$Y-specialization" is the pattern that, once composed with
;; the abstraction, will produce our desired specialization.
;; ProjectLink is a function constructor to project the (i+1)th
;; argument of an n-ary input.
;;
;; The final answer we're after is
;;
;; Compose
;;   ImplicationScope <rule-TV>
;;     VariableList
;;       Variable "$X"
;;       Variable "$Y"
;;     Evaluation
;;       Predicate "R"
;;       List
;;         Variable "$X"
;;         Variable "$Y"
;;     Evaluation
;;       Predicate "Q"
;;       Variable "$X"
;;   List
;;     Project 0
;;     Lambda
;;       VariableList
;;         Variable "$X"
;;         Variable "$Y"
;;       Inheritance
;;         Concept "a"
;;         Variable "$Y"

;; Common definitions
(define R (Predicate "R"))
(define Q (Predicate "Q"))
(define abstract-pattern
  (ImplicationScope
