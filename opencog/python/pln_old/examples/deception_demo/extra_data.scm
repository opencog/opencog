; This file is used to test intermediate inferences that guide the behavior
; of the NPC(Bob) as well as the as well as the active agent that does the
; deceving(Me).

; For testing purposes
;(EvaluationLink (stv 0.800000 0.800000)
;  (PredicateNode "Believes") ; [17]
;  (ListLink (stv 1.000000 0.000000)
;    (ConceptNode "Bob") ; [4]
;    (EvaluationLink (stv 1.000000 0.000000)
;      (PredicateNode "In") ; [29]
;      (ListLink (stv 1.000000 0.000000)
;        (ConceptNode "The_Battery") ; [41]
;        (ConceptNode "House_2") ; [42]
;      ) ; [43]
;    ) ; [44]
;  ) ; [155]
;) ; [156]


;(EvaluationLink (stv 1 1)
;    (PredicateNode "Tell")
;    (ListLink
;        (ConceptNode "Me")
;        (ConceptNode "Bob")
;        (EvaluationLink (stv 1.000000 0.000000)
;            (PredicateNode "In") ; [29]
;            (ListLink (stv 1.000000 0.000000)
;                (ConceptNode "The_Battery") ; [41]
;                (ConceptNode "House_2") ; [42]
;            ) ; [43]
;        )
;    )
;)
; For testing purposes
;(EvaluationLink (stv 1.000000 1.000000)
;  (PredicateNode "GreaterThan") ; [54]
;  (ListLink (stv 1.000000 0.000000)
;    (EvaluationLink  
;      (PredicateNode "Distance") ; [55]
;      (ListLink (stv 1.000000 0.000000)
;        (ConceptNode "Bob") ; [4]
;        (ConceptNode "House_1") ; [56]
;      ) ; [64]
;    ) ; [65]
;    (EvaluationLink  
;      (PredicateNode "Distance") ; [55]
;      (ListLink (stv 1.000000 0.000000)
;        (ConceptNode "Me") ; [1]
;        (ConceptNode "House_1") ; [56]
;      ) ; [59]
;    ) ; [60]
;  ) ; [66]
;) ; [870]

; For testing purposes
;(EvaluationLink (stv 0.800000 0.800000)
;  (PredicateNode "Believes") ; [17]
;  (ListLink (stv 1.000000 0.000000)
;    (ConceptNode "Bob") ; [4]
;    (EvaluationLink (stv 1.000000 0.000000)
;      (PredicateNode "In") ; [29]
;      (ListLink (stv 1.000000 0.000000)
;        (ConceptNode "The_Battery") ; [41]
;        (ConceptNode "House_2") ; [42]
;      ) ; [43]
;    ) ; [44]
;  ) ; [155]
;) ; [156]

; For testing purposes not mainly needed
;(EvaluationLink (stv 0.600000 0.800000)
;  (PredicateNode "GreaterThan") ; [55]
;  (ListLink
;    (EvaluationLink
;      (PredicateNode "Distance") ; [56]
;      (ListLink
;        (ConceptNode "Bob") ; [4]
;        (ConceptNode "Red_Battery") ; [69]
;      ) ; [13027]
;    ) ; [13028]
;    (EvaluationLink
;      (PredicateNode "Distance") ; [56]
;      (ListLink
;        (ConceptNode "Me") ; [1]
;        (ConceptNode "Red_Battery") ; [69]
;      ) ; [13029]
;    ) ; [13030]
;  ) ; [13031]
;) ; [20323]

; For testing
;(EvaluationLink (stv 1.000000 1.000000)
;  (PredicateNode "GreaterThan") ; [55]
;  (ListLink
;    (EvaluationLink (stv 1.000000 1.000000)
;      (PredicateNode "Distance") ; [56]
;      (ListLink
;        (ConceptNode "Bob") ; [4]
;        (ConceptNode "House_1") ; [57]
;      ) ; [65]
;    ) ; [66]
;    (EvaluationLink (stv 1.000000 1.000000)
;      (PredicateNode "Distance") ; [56]
;      (ListLink
;        (ConceptNode "Me") ; [1]
;        (ConceptNode "House_1") ; [57]
;      ) ; [60]
;    ) ; [61]
;  ) ; [67]
;) ; [17746]


; For Testing
;(EvaluationLink (stv 1.000000 1.000000)
;  (PredicateNode "Wants") ; [8]
;  (ListLink (stv 1.000000 1.000000)
;    (ConceptNode "Me") ; [4]
;    (ConceptNode "Red_Battery") ; [9]
;  ) ; [345]
;) ; [346]
