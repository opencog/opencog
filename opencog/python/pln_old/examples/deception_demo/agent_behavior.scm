; This describes the general behavior of agents, in the 3D World.

;An agent must be near something to pick it up
;(ImplicationLink (stv 1.000000 1.000000)
;    (EvaluationLink
;        (PredicateNode "PickUp")
;        (ListLink
;            (VariableNode "$A@1")
;            (VariableNode "$X@1")
;        )
;    )
;    (EvaluationLink
;        (PredicateNode "Near")
;        (ListLink
;            (VariableNode "$A@1")
;            (VariableNode "$X@1")
;        )
;    )
;)

; When the distance between Bob & House_1 is greater than the distance between
; Me and House_1. MoveTo House_1 and pick Red_Battery. (This is a plan of action)
(ImplicationLink (stv .7 .9) ; working
    (AndLink (stv 1 1)
        (EvaluationLink
          (PredicateNode "GreaterThan")
          (ListLink (stv 1.000000 0.000000)
            (EvaluationLink (stv 1.000000 1.000000)
              (PredicateNode "Distance")
              (ListLink (stv 1.000000 0.000000)
                (VariableNode "$Bob@2")
                (VariableNode "$House_1@2")
              )
            )
            (EvaluationLink
              (PredicateNode "Distance")
              (ListLink (stv 1.000000 0.000000)
                (VariableNode "$Me@2")
                (VariableNode "$House_1@2")
              )
            )
          )
        )
        (EvaluationLink
            (PredicateNode "want")
            (ListLink (stv 1.000000 0.000000)
                (VariableNode "$Me@2")
                (VariableNode "$Battery@2")
            )
        )
    )
    (AndLink
        (EvaluationLink
            (PredicateNode "MoveTo")
            (ListLink
                (VariableNode "$Me@2")
                (VariableNode "$House_1@2")
            )
        )
        (EvaluationLink
            (PredicateNode "PickUp")
            (ListLink
                (VariableNode "$Me@2")
                (VariableNode "$Battery@2") ; was red battery
            )
        )
    )
)

; If agent B and A are equal distances from some object C and if agent A is told
; to move to postion L then distance between agent A and C is far (We assume that
; the other agent is obedient).
(ImplicationLink (stv 1 1) ; working
    (AndLink (stv 1 1)
        (EvaluationLink
            (PredicateNode "MoveTo")
            (ListLink
                (VariableNode "$Bob@3")
                (VariableNode "$House_2@3")
            )
        )
        (EvaluationLink
            (PredicateNode "EqualTo")
            (ListLink
                (EvaluationLink
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$Bob@3")
                        (VariableNode "$House_1@3")
                    )
                )
                (EvaluationLink (stv 1 1)
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$Me@3")
                        (VariableNode "$House_1@3")
                    )
                )
            )
        )
        (EvaluationLink
            (PredicateNode "GreaterThan")
            (ListLink
                (EvaluationLink
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$House_1@3")
                        (VariableNode "$House_2@3")
                    )
                )
                (EvaluationLink
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$Me@3")
                        (VariableNode "$House_1@3")
                    )
                )
            )
        )
    )
    (EvaluationLink (stv 1.000000 1.000000)
      (PredicateNode "GreaterThan")
      (ListLink (stv 1.000000 0.000000)
        (EvaluationLink (stv 1.000000 1.000000)
          (PredicateNode "Distance")
          (ListLink (stv 1.000000 0.000000)
            (VariableNode "$Bob@3")
            (VariableNode "$House_1@3")
          )
        )
        (EvaluationLink (stv 1.000000 1.000000)
          (PredicateNode "Distance")
          (ListLink (stv 1.000000 0.000000)
            (VariableNode "$Me@3")
            (VariableNode "$House_1@3")
          )
        )
      )
    )
)
