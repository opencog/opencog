(use-modules (opencog logger))
(cog-logger-set-level! "debug")

(load "pln-config.scm")

(MemberLink (stv 1.000000 1.000000)
  (ConceptNode "Org1-1" (stv 0.010000 1.000000))
  (ConceptNode "Organization" (stv 0.010000 1.000000))
)

(define target
(ExistsLink
  (TypedVariableLink
    (VariableNode "?MEMBER")
    (TypeChoice
      (TypeNode "ConceptNode")
      (TypeNode "SchemaNode")
      (TypeNode "PredicateNode")
    )
  )
  (MemberLink
    (VariableNode "?MEMBER")
    (ConceptNode "Org1-1" (stv 0.010000 1.000000))
  )
)
)

(pln-bc target)
