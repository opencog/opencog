;; Knowledge-base for the universal instantiation example

;; All concepts inherits from the abstractverse
(define forall
(ForAll (stv 1 1)
  (TypedVariable
    (Variable "$C")
    (Type "ConceptNode"))
  (Inheritance
    (Variable "$C")
    (Concept "Abstractverse")))
)

;; Instances of concepts
(Concept "Cat")
(Concept "Time")
(Concept "Space")
(Concept "Infinity")
(Concept "Mathematics")
