; Major premise:  The jar is filled with yellow marbles
; Minor premise:  I have a yellow marble in my hand
; |-
; The yellow marble was taken out of the jar

(EvaluationLink (PredicateNode "inputs")
    (ListLink
        ; "The jar is filled with yellow marbles"
        ; current RelEx2Logic representation
        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "filled@56599458-637a-4c3c-9276-6f16a5447618") ; [337]
          (PredicateNode "fill") ; [338]
        ) ; [339]
        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "filled@56599458-637a-4c3c-9276-6f16a5447618") ; [337]
          (ListLink (stv 0.990000 0.990000)
            (VariableNode "$x") ; [345]
            (ConceptNode "jar@cf3704c1-4717-4999-b7cd-d18923b6d071") ; [342]
          ) ; [346]
        ) ; [347]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "jar@cf3704c1-4717-4999-b7cd-d18923b6d071") ; [342]
          (ConceptNode "jar") ; [343]
        ) ; [344]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "yellow@de7a5383-a6d6-4538-b53c-da81884fb8c0") ; [351]
          (ConceptNode "yellow") ; [352]
        ) ; [353]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "marbles@0e7a9636-4725-4207-a48a-ea9f0f639ccc") ; [354]
          (ConceptNode "yellow@de7a5383-a6d6-4538-b53c-da81884fb8c0") ; [351]
        ) ; [355]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "marbles@0e7a9636-4725-4207-a48a-ea9f0f639ccc") ; [354]
          (ConceptNode "marbles") ; [356]
        ) ; [357]
        ; "I have a yellow marble in my hand"
        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "have@ce8fbf1e-18a3-49d3-9873-81ebc76e7b4f") ; [511]
          (PredicateNode "have") ; [512]
        ) ; [513]
        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "Possession") ; [533]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "hand@b4bdb455-6175-4d12-b09c-ae580db3c14c") ; [527]
            (ConceptNode "my@26aaee01-0701-4c7d-82cc-0bffb5b7aec0") ; [530]
          ) ; [534]
        ) ; [535]
        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "have@ce8fbf1e-18a3-49d3-9873-81ebc76e7b4f") ; [511]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "I@6603e246-86ac-482d-9af7-9b1b6514d619") ; [514]
            (ConceptNode "marble@8d369547-8465-4809-93f5-e8bd9239b5d2") ; [517]
          ) ; [520]
        ) ; [521]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "marble@8d369547-8465-4809-93f5-e8bd9239b5d2") ; [517]
          (ConceptNode "marble") ; [518]
        ) ; [519]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "hand@b4bdb455-6175-4d12-b09c-ae580db3c14c") ; [527]
          (ConceptNode "hand") ; [528]
        ) ; [529]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "my@26aaee01-0701-4c7d-82cc-0bffb5b7aec0") ; [530]
          (ConceptNode "me") ; [531]
        ) ; [532]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "yellow@5094c947-c5c3-4e0c-935a-7d8885e9d920") ; [540]
          (ConceptNode "yellow") ; [541]
        ) ; [542]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "marble@8d369547-8465-4809-93f5-e8bd9239b5d2") ; [517]
          (ConceptNode "yellow@5094c947-c5c3-4e0c-935a-7d8885e9d920") ; [540]
        ) ; [543]
        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "I@6603e246-86ac-482d-9af7-9b1b6514d619") ; [514]
          (ConceptNode "I") ; [515]
        ) ; [516]
    )
)

(EvaluationLink (PredicateNode "rules")
    (ListLink
        (ConceptNode "AbductionRule")
    )
)

(EvaluationLink (PredicateNode "desired_outputs")
    (ListLink
        ; "The yellow marble was taken out of the jar"
)
