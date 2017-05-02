(use-modules (opencog logger))
(use-modules (opencog rule-engine))

(cog-logger-set-level! "fine")

(add-to-load-path ".")
(load-from-path "deduction-rule.scm")

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "tom")
    (ConceptNode "human"))

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "human")
    (ConceptNode "speak"))

(define source
(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "tom")
    (ConceptNode "human")))

(MemberLink (stv 0.5 1)
  deduction-implication-rule-name
  (ConceptNode "rule-base")
)

(MemberLink (stv 0.5 1)
  deduction-subset-rule-name
  (ConceptNode "rule-base")
  )

(define base (ConceptNode "rule-base"))

(InheritanceLink  
  (ConceptNode "rule-base")
  (ConceptNode "URE")
)

(ExecutionLink
   (SchemaNode "URE:maximum-iterations")
   (ConceptNode "rule-base")
   (NumberNode 20)
)

(MemberLink (stv 0.9 1)
  deduction-inheritance-rule-name
  (ConceptNode "rule-base")
  )

(cog-fc base source (List) (SetLink))
