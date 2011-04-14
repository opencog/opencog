
(define (stv mean conf) (cog-new-stv mean conf))

; Some data to populate the atomspace
(AssociativeLink (stv 1.0 1.0)
   (ConceptNode "want-this")
   (ConceptNode "valid")
)

(AssociativeLink (stv 1.0 1.0)
   (ConceptNode "want-this")
   (ConceptNode "one-high, 4-arity")
   (ConceptNode "mehh2")
   (ConceptNode "mehh3")
)

(AssociativeLink (stv 1.0 1.0)
   (ConceptNode "want-this")
   (ListLink
      (WordInstanceNode "color")
      (WordInstanceNode "blue")
   )
)

(define (untyped-link-match)
   (BindLink
      (ListLink 
         (VariableNode "$var")
      )
      (ImplicationLink
         (AndLink
            (AssociativeLink (stv 1 0.99999988)
               (ConceptNode "want-this")
               (VariableNode "$var")
            )
         )
         (ListLink (stv 1 0.99999988)
            (VariableNode "$var")
         )
      )
   )
)


