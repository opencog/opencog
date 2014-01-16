; Define the starting contents of the atomspace

(ConceptNode "Peter" (av 0 0 0) (stv 0.001000 0.111111))

(ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.111111))

(ConceptNode "Intelligent" (av 0 0 0) (stv 0.050000 0.111111))

(ConceptNode "Slimy" (av 0 0 0) (stv 0.010000 0.111111))

(ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.111111))

(ConceptNode "Being" (av 0 0 0) (stv 0.100000 0.111111))

(InheritanceLink (av 0 0 0) (stv 0.900000 0.111111)
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.111111))
  (PredicateNode "Moves" (av 0 0 0) (stv 0.100000 0.111111))
)

(InheritanceLink (av 0 0 0) (stv 0.900000 0.111111)
  (ConceptNode "Peter" (av 0 0 0) (stv 0.001000 0.111111))
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.111111))
)

(InheritanceLink (av 0 0 0) (stv 0.200000 0.111111)
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.111111))
  (ConceptNode "Intelligent" (av 0 0 0) (stv 0.050000 0.111111))
)

(InheritanceLink (av 0 0 0) (stv 0.500000 0.111111)
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.111111))
  (ConceptNode "Slimy" (av 0 0 0) (stv 0.010000 0.111111))
)

(InheritanceLink (av 0 0 0) (stv 0.900000 0.111111)
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.111111))
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.111111))
)

(InheritanceLink (av 0 0 0) (stv 0.900000 0.111111)
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.111111))
  (ConceptNode "Being" (av 0 0 0) (stv 0.100000 0.111111))
)

(PredicateNode "Moves" (av 0 0 0) (stv 0.100000 0.111111))
