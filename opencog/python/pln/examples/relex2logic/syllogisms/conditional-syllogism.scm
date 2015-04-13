#|
Note: Conditional syllogisms are better known as hypothetical syllogisms,
because the arguments used here are not always valid. The basic of this
syllogism type is: if A is true then B is true as well.
|#

(EvaluationLink (PredicateNode "inputs")
    (ListLink
        ; Major premise: "If Johnny is eating sweets every day, he is placing
        ; himself at risk for diabetes."
        ; Current RelEx & RelEx2Logic output:
        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "is@017b204f-62c4-4d40-aa5a-604c735979bc") ; [1102]
          (PredicateNode "be") ; [1103]
        ) ; [1104]

        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "placing@2d8ddb4f-dcb3-43c9-8247-1775208b44ee") ; [1085]
          (PredicateNode "place") ; [1086]
        ) ; [1087]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1082]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "he@6b863a70-f21a-43f8-bd8f-cebe3c1bc685") ; [1079]
          ) ; [1083]
        ) ; [1084]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "placing@2d8ddb4f-dcb3-43c9-8247-1775208b44ee") ; [1085]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "he@6b863a70-f21a-43f8-bd8f-cebe3c1bc685") ; [1079]
            (ConceptNode "himself@7076068f-4f5e-4686-8dab-b29879b02ef6") ; [1088]
          ) ; [1091]
        ) ; [1092]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1082]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "Johnny@0d917209-b7ab-400e-b456-f457842d1c58") ; [1095]
          ) ; [1120]
        ) ; [1121]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1082]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "himself@7076068f-4f5e-4686-8dab-b29879b02ef6") ; [1088]
          ) ; [1122]
        ) ; [1123]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "he@6b863a70-f21a-43f8-bd8f-cebe3c1bc685") ; [1079]
          (ConceptNode "he") ; [1080]
        ) ; [1081]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "himself@7076068f-4f5e-4686-8dab-b29879b02ef6") ; [1088]
          (ConceptNode "himself") ; [1089]
        ) ; [1090]

        (InheritanceLink (stv 0.990000 0.990000)
          (PredicateNode "placing@2d8ddb4f-dcb3-43c9-8247-1775208b44ee") ; [1085]
          (ConceptNode "present_progressive") ; [1093]
        ) ; [1094]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "Johnny@0d917209-b7ab-400e-b456-f457842d1c58") ; [1095]
          (ConceptNode "Johnny") ; [1096]
        ) ; [1097]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "sweets@6961464c-b499-485d-ae61-d1b64ecc3429") ; [1098]
          (ConceptNode "sweet") ; [1099]
        ) ; [1100]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "Johnny@0d917209-b7ab-400e-b456-f457842d1c58") ; [1095]
          (ConceptNode "sweets@6961464c-b499-485d-ae61-d1b64ecc3429") ; [1098]
        ) ; [1101]

        (InheritanceLink (stv 0.990000 0.990000)
          (PredicateNode "is@017b204f-62c4-4d40-aa5a-604c735979bc") ; [1102]
          (ConceptNode "present") ; [1105]
        ) ; [1106]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "day@8eebf3fb-27c9-41e8-97c9-f1c00003989a") ; [1107]
          (ConceptNode "day") ; [1108]
        ) ; [1109]

        (InheritanceLink (stv 0.990000 0.990000)
          (SatisfyingSetLink (stv 0.990000 0.990000)
            (PredicateNode "is@017b204f-62c4-4d40-aa5a-604c735979bc") ; [1102]
          ) ; [1110]
          (ConceptNode "day@8eebf3fb-27c9-41e8-97c9-f1c00003989a") ; [1107]
        ) ; [1111]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "eating@bece4386-2a1d-4d88-8a17-bc7657154959") ; [1112]
          (ConceptNode "eating") ; [1113]
        ) ; [1114]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "sweets@6961464c-b499-485d-ae61-d1b64ecc3429") ; [1098]
          (ConceptNode "eating@bece4386-2a1d-4d88-8a17-bc7657154959") ; [1112]
        ) ; [1115]

        (InheritanceLink (stv 0.990000 0.990000)
          (SpecificEntityNode "Johnny@0d917209-b7ab-400e-b456-f457842d1c58") ; [1116]
          (ConceptNode "Johnny") ; [1096]
        ) ; [1117]

        (InheritanceLink (stv 0.990000 0.990000)
          (SpecificEntityNode "Johnny@0d917209-b7ab-400e-b456-f457842d1c58") ; [1116]
          (ConceptNode "person") ; [1118]
        ) ; [1119]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "every@5f453a5b-377c-4917-bfb4-72961fee164b") ; [1124]
          (ConceptNode "every") ; [1125]
        ) ; [1126]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "day@8eebf3fb-27c9-41e8-97c9-f1c00003989a") ; [1107]
          (ConceptNode "every@5f453a5b-377c-4917-bfb4-72961fee164b") ; [1124]
        ) ; [1127]

        ; Minor premise: "Johnny does not eat sweats every day."
        ; Current RelEx & RelEx2Logic output:

        (ImplicationLink (stv 0.990000 0.990000)
          (PredicateNode "eat@b3fac38d-b546-439f-ae68-1ac1b00d812a") ; [1296]
          (PredicateNode "eat") ; [1297]
        ) ; [1298]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "definite") ; [1286]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "Johnny@1e78390b-c0d2-40c0-9ff1-f44ec9dfc655") ; [1284]
          ) ; [1287]
        ) ; [1288]

        (EvaluationLink (stv 0.990000 0.990000)
          (PredicateNode "eat@b3fac38d-b546-439f-ae68-1ac1b00d812a") ; [1296]
          (ListLink (stv 0.990000 0.990000)
            (ConceptNode "Johnny@1e78390b-c0d2-40c0-9ff1-f44ec9dfc655") ; [1284]
            (ConceptNode "sweets@c9f42575-3bfc-4884-910b-b2a09652a8ef") ; [1299]
          ) ; [1302]
        ) ; [1303]

        (InheritanceLink (stv 0.990000 0.990000)
          (SpecificEntityNode "Johnny@1e78390b-c0d2-40c0-9ff1-f44ec9dfc655") ; [1280]
          (ConceptNode "Johnny") ; [1279]
        ) ; [1281]

        (InheritanceLink (stv 0.990000 0.990000)
          (SpecificEntityNode "Johnny@1e78390b-c0d2-40c0-9ff1-f44ec9dfc655") ; [1280]
          (ConceptNode "person") ; [1282]
        ) ; [1283]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "Johnny@1e78390b-c0d2-40c0-9ff1-f44ec9dfc655") ; [1284]
          (ConceptNode "Johnny") ; [1279]
        ) ; [1285]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "every@98cd07f9-4ff6-4c88-9406-bef62bf30173") ; [1289]
          (ConceptNode "every") ; [1290]
        ) ; [1291]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "day@72231824-7119-4d31-99ac-ec1ec92d3cee") ; [1292]
          (ConceptNode "every@98cd07f9-4ff6-4c88-9406-bef62bf30173") ; [1289]
        ) ; [1293]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "day@72231824-7119-4d31-99ac-ec1ec92d3cee") ; [1292]
          (ConceptNode "day") ; [1294]
        ) ; [1295]

        (InheritanceLink (stv 0.990000 0.990000)
          (ConceptNode "sweets@c9f42575-3bfc-4884-910b-b2a09652a8ef") ; [1299]
          (ConceptNode "sweet") ; [1300]
        ) ; [1301]

        (InheritanceLink (stv 0.990000 0.990000)
          (PredicateNode "eat@b3fac38d-b546-439f-ae68-1ac1b00d812a") ; [1296]
          (ConceptNode "present_infinitive") ; [1304]
        ) ; [1305]

        (InheritanceLink (stv 0.990000 0.990000)
          (SatisfyingSetLink (stv 0.990000 0.990000)
            (PredicateNode "eat@b3fac38d-b546-439f-ae68-1ac1b00d812a") ; [1296]
          ) ; [1306]
          (ConceptNode "day@72231824-7119-4d31-99ac-ec1ec92d3cee") ; [1292]
        ) ; [1307]
    )
)

(EvaluationLink (PredicateNode "rules")
    (ListLink
        ; the rules should be listed here
    )
)

(EvaluationLink (PredicateNode "desired_outputs")
    (ListLink
        ; "Therefore Johnny is not placing himself at risk for diabetes."
)

#|
Note: This conclusion is invalid because it is possible that Johnny does not
eat sweats every day but does eats cake every day what also puts him at risk
for diabetes.
|#
