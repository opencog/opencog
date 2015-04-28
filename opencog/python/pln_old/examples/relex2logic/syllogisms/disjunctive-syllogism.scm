#|
These syllogism types do not actually state that a certain premise
(major or minor) is correct, but is does states that one of the premises is
correct. The basic type for this syllogism is: Either A or B is true, but they
can’t be true at the same time.
|#

(EvaluationLink (PredicateNode "inputs")
    ; Major premise: "Either the meeting is at school or at home."
    ; Current RelEx & RelEx2Logic output:
    (ListLink
        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "is@cf2062c3-efa0-4d1e-b75e-f5bafdce4d04") ; [1473]
          (PredicateNode "be") ; [1474]
        ) ; [1475]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1470]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "meeting@5d6d7498-c7fe-4738-95e2-9620d00d478e") ; [1467]
          ) ; [1471]
        ) ; [1472]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "is@cf2062c3-efa0-4d1e-b75e-f5bafdce4d04") ; [1473]
          (ConceptNode "meeting@5d6d7498-c7fe-4738-95e2-9620d00d478e") ; [1467]
        ) ; [1478]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "meeting@5d6d7498-c7fe-4738-95e2-9620d00d478e") ; [1467]
          (ConceptNode "meeting") ; [1468]
        ) ; [1469]

        (InheritanceLink (stv 0.990000 0.990000)
          (PredicateNode "is@cf2062c3-efa0-4d1e-b75e-f5bafdce4d04") ; [1473]
          (ConceptNode "present") ; [1476]
        ) ; [1477]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "at@a1a09c79-0e59-4b09-b29a-6fb90f2bd3c3") ; [1479]
          (ConceptNode "at") ; [1480]
        ) ; [1481]

        (InheritanceLink (stv 0.990000 0.990000)
          (SatisfyingSetLink (stv 0.990000 0.990000)
            (PredicateNode "is@cf2062c3-efa0-4d1e-b75e-f5bafdce4d04") ; [1473]
          ) ; [1482]
          (ConceptNode "at@a1a09c79-0e59-4b09-b29a-6fb90f2bd3c3") ; [1479]
        ) ; [1483]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "at@728befb2-8674-44d8-ba5a-cded7d2d8b03") ; [1484]
          (ConceptNode "at") ; [1480]
        ) ; [1485]

        (InheritanceLink (stv 0.990000 0.990000)
          (SatisfyingSetLink (stv 0.990000 0.990000)
            (PredicateNode "is@cf2062c3-efa0-4d1e-b75e-f5bafdce4d04") ; [1473]
          ) ; [1482]
          (ConceptNode "at@728befb2-8674-44d8-ba5a-cded7d2d8b03") ; [1484]
        ) ; [1486]

        ; Minor premise: "The meeting is not at home."
        ; Current RelEx & RelEx2Logic output:

        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "at@92159926-51f7-4b46-bae7-32ee648fc6ea") ; [1629]
          (PredicateNode "at") ; [1630]
        ) ; [1631]

        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "is@1d2ea7ff-28fa-47f5-a048-d4b1ac096067") ; [1618]
          (PredicateNode "be") ; [1621]
        ) ; [1622]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1626]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "meeting@93b8fb75-0e7a-4809-9b9c-58cc6bae20a8") ; [1623]
          ) ; [1627]
        ) ; [1628]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "not@e9be8c0d-b794-4264-97c8-c20789765ef7") ; [1615]
          (ConceptNode "not") ; [1616]
        ) ; [1617]

        (InheritanceLink (stv 0.990000 0.990000)
          (SatisfyingSetLink (stv 0.990000 0.990000)
            (PredicateNode "is@1d2ea7ff-28fa-47f5-a048-d4b1ac096067") ; [1618]
          ) ; [1619]
          (ConceptNode "not@e9be8c0d-b794-4264-97c8-c20789765ef7") ; [1615]
        ) ; [1620]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "meeting@93b8fb75-0e7a-4809-9b9c-58cc6bae20a8") ; [1623]
          (ConceptNode "meeting") ; [1624]
        ) ; [1625]

        (InheritanceLink (stv 0.990000 0.990000)
          (PredicateNode "at@92159926-51f7-4b46-bae7-32ee648fc6ea") ; [1629]
          (ConceptNode "present") ; [1632]
        ) ; [1633]
    )
)

(EvaluationLink (PredicateNode "rules")
    (ListLink
        ; the rules should be listed here
    )
)

(EvaluationLink (PredicateNode "desired_outputs")
    (ListLink
        ; "Therefore the meeting is at school."
)
