opencog> py-eval pln_examples.run_pln_example('../tests/python/test_pln/scm/specific_rules/DeductionRule2.scm')
../tests/python/test_pln/scm/specific_rules/DeductionRule2.scm
(InheritanceLink (stv 1.000000 0.000000)
  (ConceptNode "Kermit")
  (ConceptNode "Animal")
)

[(ConceptNode "DeductionRule" (av 0 0 0) (stv 1.000000 0.000000)) ; Handle: 70
]
['DeductionRule (InheritanceLink InheritanceLink VariableNode VariableNode -> InheritanceLink)']
Trying to produce truth values for atom:
(InheritanceLink (av 0 0 0) (stv 1.000000 0.000000) ; Handle: 65
  (ConceptNode "Kermit" (av 0 0 0) (stv 0.001000 0.900000)) ; Handle: 60
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
)

(<pln.rules.rules.DeductionRule object at 0x5579bd0>, [(InheritanceLink (av 0 0 0) (stv 0.900000 0.900000) ; Handle: 64
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.900000)) ; Handle: 61
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
)
, (InheritanceLink (av 0 0 0) (stv 1.000000 0.000000) ; Handle: 542
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
)
, (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
, (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
], [(InheritanceLink (av 0 0 0) (stv 0.900000 0.900000) ; Handle: 64
  (ConceptNode "Frog" (av 0 0 0) (stv 0.010000 0.900000)) ; Handle: 61
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
)
])
Target produced!
(InheritanceLink (av 0 0 0) (stv 0.811010 0.890110) ; Handle: 65
  (ConceptNode "Kermit" (av 0 0 0) (stv 0.001000 0.900000)) ; Handle: 60
  (ConceptNode "Animal" (av 0 0 0) (stv 0.100000 0.900000)) ; Handle: 62
)

Inference steps

Step 1
Premise <UUID:63> (InheritanceLink (stv 0.900000 0.900000)
  (ConceptNode "Kermit")
  (ConceptNode "Frog")
)


Step 2
Premise <UUID:62> (ConceptNode "Animal")


Step 3
Premise <UUID:61> (ConceptNode "Frog")


Step 4
Premise <UUID:64> (InheritanceLink (stv 0.900000 0.900000)
  (ConceptNode "Frog")
  (ConceptNode "Animal")
)


Step 5
<UUID:64> (InheritanceLink (stv 0.900000 0.900000)
  (ConceptNode "Frog")
  (ConceptNode "Animal")
)

<UUID:61> (ConceptNode "Frog")

<UUID:62> (ConceptNode "Animal")

<UUID:63> (InheritanceLink (stv 0.900000 0.900000)
  (ConceptNode "Kermit")
  (ConceptNode "Frog")
)

|=
<UUID:65> (InheritanceLink (stv 0.811010 0.890110)
  (ConceptNode "Kermit")
  (ConceptNode "Animal")
)

None
