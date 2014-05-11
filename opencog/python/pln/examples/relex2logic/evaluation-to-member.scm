; Socrates is a man.
(ConceptNode "Socrates" (stv .001 1))
(ConceptNode "man" (stv .01 1))
(ConceptNode "air" (stv .01 1))

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "Socrates")
    (ConceptNode "man"))

; Men breathe air.
(EvaluationLink (stv 1.0 1.0)
  (PredicateNode "breathe")
  (ListLink
    (ConceptNode "man")
    (ConceptNode "air")))
