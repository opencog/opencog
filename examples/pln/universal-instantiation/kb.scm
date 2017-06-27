;; Knowledge-base of the universal instantiation example
;;
;; Says all concepts inherits from the abstractiverse

(define forall
(ForAll (stv 1 1)
  (TypedVariable
    (Variable "$C")
    (Type "ConceptNode"))
  (Inheritance
    (Variable "$C")
    (Concept "Abstractiverse")))
)

;; Instances of concepts

(Concept "Cat")
(Concept "Time")
(Concept "Space")
(Concept "Infinity")
(Concept "Mathematics")
