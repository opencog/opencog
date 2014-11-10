;; The following are rules that define the physics (so to speak)
;; of the world as well as general rules about spatial-relations.

; The distance between Bob and House_1 is approximately the distance between Bob
; and Red_Battery if Red_Battery is in the House_1.
(ImplicationLink (stv .5 .9) ; working
    (AndLink (stv 1 1)
        (EvaluationLink
            (PredicateNode "Distance")
            (ListLink
                (VariableNode "$Bob@10")
                (VariableNode "$House_1@10")
            )
        )
        (EvaluationLink
            (PredicateNode "In")
            (ListLink
                (VariableNode "$Red_Battery@10")
                (VariableNode "$House_1@10")
            )
        )
    )
    (EvaluationLink
        (PredicateNode "Distance")
        (ListLink
            (VariableNode "$Bob@10")
            (VariableNode "$Red_Battery@10")
        )
    )
)

; The distance relation between agents and an object(eg House) is approximately equal to
; the distance realtion between agents and an object in the previous object (eg. Battery)
(ImplicationLink (stv .6 .8) ; working
    (AndLink (stv 1 1)
        (EvaluationLink
            (VariableNode "$MagnitudeRelation@11")
            (ListLink
                (EvaluationLink  
                  (PredicateNode "Distance")
                  (ListLink
                    (VariableNode "$Bob@11")
                    (VariableNode "$House_1@11")
                  )
                )
                (EvaluationLink  
                  (PredicateNode "Distance")
                  (ListLink
                    (VariableNode "$Me@11")
                    (VariableNode "$House_1@11")
                  )
                )
            )
        )
        (EvaluationLink (stv 1 1)
            (PredicateNode "In")
            (ListLink
                (VariableNode "$Red_Battery@11")
                (VariableNode "$House_1@11")
            )
        )
    )
    (EvaluationLink  
        (VariableNode "$MagnitudeRelation@11")
        (ListLink
            (EvaluationLink
              (PredicateNode "Distance")
              (ListLink
                (VariableNode "$Bob@11")
                (VariableNode "$Red_Battery@11")
              )
            )
            (EvaluationLink 
              (PredicateNode "Distance")
              (ListLink
                (VariableNode "$Me@11")
                (VariableNode "$Red_Battery@11")
              )
            )
        )
    )
)


; Notes; Frame of reference translation rules are required; specially for simulating
; the actions of other agents. Why simulate? Cause it helps in exploring possible
; alternatives in planning behaviors of the character/robot.

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

