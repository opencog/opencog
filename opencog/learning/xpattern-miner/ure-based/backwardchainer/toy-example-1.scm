;; Let be a binary predicate P. The pattern we want to discover is
;;
;; P(Inheritance(X, Y))
;;
;; starting from the abstraction
;;
;; P(X)
;;
;; Given a number of instances of P
;;
;; That is, in atomese, the representation of the abstraction is
;;
;; Lambda
;;   Variable "$X"
;;   Evaluation
;;     Predicate "P"
;;     Variable "$X"
;;
;; and the representation of the specialization is
;;
;; Lambda
;;   VariableList
;;     Variable "$X"
;;     Variable "$Y"
;;   Evaluation
;;     Predicate "P"
;;     Inheritance
;;       Variable "$X"
;;       Variable "$Y"
;;
;; For that we build the backward chainer query
;;
;; Compose
;;   Lambda
;;     Variable "$X"
;;     Evaluation
;;       Predicate "P"
;;       Variable "$X"
;;   Variable "$X-specialization"
;;
;; where "$X-specialization" is the pattern that, once composed with
;; the abstraction, will produce our desired specialization.
;;
;; The final answer we're after is
;;
;; Compose
;;   Lambda
;;     Variable "$X"
;;     Evaluation
;;       Predicate "P"
;;       Variable "$X"
;;   Lambda
;;     VariableList
;;       Variable "$X"
;;       Variable "$Y"
;;     Inheritance
;;       Variable "$X"
;;       Variable "$Y"

;; Common definitions
(define P (Predicate "P"))
(define X (Variable "$X"))
(define Y (Variable "$Y"))
(define minsup (Predicate "minsup"))

;; Text
(define A (Concept "A"))
(define B (Concept "B"))
(define C (Concept "C"))
(define AB (Inheritance A B))
(define BC (Inheritance B C))
(define AC (Inheritance A C))
(define PAB (Evaluation P AB))
(define PBC (Evaluation P BC))
(define PAC (Evaluation P AC))

;; Abstraction
(define abstract
  (Lambda X (Evaluation P X)))

;; Specialization
(define specialization
  (Lambda (Inheritance X Y)))

;; Abstraction has enough support
(Evaluation (stv 1 1) minsup (List abstract (Number 3)))

;; Query
(define X-special (Variable "$X-special"))
(define query
  (Evaluation minsup (List (Compose abstract X-special) (Number 3))))
