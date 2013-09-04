;#; generated from fetchdemo5.xml then cleaned at hand
;#
;#; concepts
;#(define ball (ConceptNode "ball"))
;#(define teacher (ConceptNode "teacher"))
;#
;#; actions
;#(define walktowards (SchemaNode "walktowards"))
;#(define give (SchemaNode "give"))
;#(define hold (SchemaNode "hold"))
;#(define lift (SchemaNode "lift"))
;#
;#; variables
;#(define Vobj (VariableNode "$obj"))
;#(define V10000 (VariableNode "$10000"))
;#(define V1 (VariableNode "$1"))
;#(define V2 (VariableNode "$2"))
;#
;#; predicates
;#(define PPP (PredicateNode "+++"))
;#(define teacher_say (PredicateNode "teacher_say"))
;#(define just_done (PredicateNode "just_done"))
;#(define near (PredicateNode "near"))
;#(define do (PredicateNode "do"))
;#(define can_do (PredicateNode "can_do"))
;#
;#; words
;#(define fetch (WordNode "fetch"))
;#
;#; lists (predicate's arguments)
;#(define L_ball (ListLink ball))
;#(define L_teacher (ListLink teacher))
;#(define L_Vobj (ListLink Vobj))
;#(define L_V1 (ListLink V1))
;#(define L_V2 (ListLink V2))
;#(define L_ball_teacher (ListLink ball teacher))
;#
;#; define evaluations
;#(define walktowards_ball (EvaluationLink walktowards L_ball))
;#(define walktowards_teacher (EvaluationLink walktowards L_teacher))
;#(define walktowards_V2 (EvaluationLink walktowards L_V2))
;#(define give_ball_teacher (EvaluationLink give L_ball_teacher))
;#(define hold_ball (EvaluationLink hold L_ball))
;#(define lift_Vobj (EvaluationLink lift L_Vobj))
;#(define lift_ball (EvaluationLink lift L_ball))
;#
;#(AndLink (EvaluationLink just_done
;#                         (ListLink hold_ball))
;#         (EvaluationLink near
;#                         L_teacher))
;#
;#(AndLink (EvaluationLink (stv 0.99000001 0.99000001) teacher_say
;#                         (ListLink fetch))
;#         (EvaluationLink just_done
;#                         (ListLink give_ball_teacher)))
;#
;#(AndLink (EvaluationLink just_done
;#                         (ListLink walktowards_V2)))
;#
;#(AndLink (EvaluationLink do
;#                         L_V1)
;#         (EvaluationLink can_do
;#                         L_V1))
;#
;#(ForAllLink (stv 1 0.80000001) L_Vobj
;#            (ImplicationLink (EvaluationLink near
;#                                             L_Vobj)
;#                             (EvaluationLink can_do
;#                                             (ListLink lift_Vobj))))
;#
;#(ForAllLink (stv 1 0.99900001) (ListLink V10000)
;#            (EvaluationLink do
;#                            (ListLink V10000)))
;#
;#(ImplicationLink (EvaluationLink near
;#                                 L_Vobj)
;#                 (EvaluationLink can_do
;#                                 (ListLink lift_Vobj)))
;#
;#(ImplicationLink (stv 1 0.99000001) (AndLink (EvaluationLink just_done
;#                                                             (ListLink hold_ball))
;#                                             (EvaluationLink near
;#                                                             L_teacher))
;#                 (EvaluationLink can_do
;#                                 (ListLink give_ball_teacher)))
;#
;#(ImplicationLink (stv 1 0.99000001) (EvaluationLink just_done
;#                                                    (ListLink lift_ball))
;#                 (EvaluationLink can_do
;#                                 (ListLink hold_ball)))
;#
;#(ImplicationLink (stv 1 0.80000001) (AndLink (EvaluationLink (stv 0.99000001 0.99000001) teacher_say
;#                                                             (ListLink fetch))
;#                                             (EvaluationLink just_done
;#                                                             (ListLink give_ball_teacher)))
;#                 (EvaluationLink PPP))
;#
;#(ImplicationLink (stv 1 0.99900001) (AndLink (EvaluationLink just_done
;#                                                             (ListLink walktowards_V2)))
;#                 (EvaluationLink near
;#                                 L_V2))
;#
;#(ImplicationLink (stv 1 0.99900001) (AndLink (EvaluationLink do
;#                                                             L_V1)
;#                                             (EvaluationLink can_do
;#                                                             L_V1))
;#                 (EvaluationLink just_done
;#                                 L_V1))
;#
;#(EvaluationLink just_done
;#                (ListLink give_ball_teacher))
;#
;#(EvaluationLink PPP)
;#
;#(EvaluationLink just_done
;#                (ListLink hold_ball))
;#
;#(EvaluationLink near
;#                L_teacher)
;#
;#(EvaluationLink can_do
;#                (ListLink give_ball_teacher))
;#
;#walktowards_V2
;#
;#(EvaluationLink just_done
;#                (ListLink walktowards_V2))
;#
;#(EvaluationLink near
;#                L_V2)
;#
;#walktowards_teacher
;#
;#(EvaluationLink (stv 1 0.99000001) can_do
;#                (ListLink walktowards_teacher))
;#
;#lift_ball
;#
;#walktowards_ball
;#
;#(EvaluationLink just_done
;#                (ListLink lift_ball))
;#
;#(EvaluationLink (stv 1 0.99000001) can_do
;#                (ListLink walktowards_ball))
;#
;#hold_ball
;#
;#(EvaluationLink can_do
;#                (ListLink hold_ball))
;#
;#(EvaluationLink near
;#                L_Vobj)
;#
;#lift_Vobj
;#
;#(EvaluationLink can_do
;#                (ListLink lift_Vobj))
;#
;#(EvaluationLink do
;#                (ListLink V10000))
;#
;#(EvaluationLink do
;#                L_V1)
;#
;#(EvaluationLink can_do
;#                L_V1)
;#
;#(EvaluationLink just_done
;#                L_V1)
;#
;#(EvaluationLink (stv 0.99000001 0.99000001) teacher_say
;#                (ListLink fetch))
;#
;#give
;#hold
;#walktowards
;#lift
;#PPP
;#just_done
;#near
;#do
;#can_do
;#teacher_say
;#fetch
;#V2
;#V1


; For ModusPonensRule (in code: StrictImplicationBreakdownRule),
; using ImplicationBreakdownFormula.
; This Rule only works when the premises are Links, not Nodes; I'm not sure
; there's a good reason for that. The code for the Rule and the Formula are
; both weird; I'm not quite sure what atoms it uses and how. -- JaredW

(define eval_a (EvaluationLink (PredicateNode "A") (stv 0.5 0.8)))   ; premise #2
(define eval_b (EvaluationLink (PredicateNode "B")))                 ; conclusion
(define imp_eva_evb (ImplicationLink eval_a eval_b (stv 0.85 0.79))) ; premise #1

; EnergyGoal (a Demand Goal, also known as Final Goal)
(define EnergyGoal
    (EvaluationLink
        (PredicateNode "EnergyGoal") 
    ) 
)


; Intermediate Goals, which can be used as Preconditions
(define GetFoodGoal
    (EvaluationLink 
        (PredicateNode "GetFoodGoal")  
    ) 
)

(define LunchTime
    (EvaluationLink (stv 0.8 1.0)
        (PredicateNode "LunchTime") 
    )
)




(define FindFoodGoal
    (EvaluationLink (stv 0.9 1.0)
        (PredicateNode "FindFoodGoal") 
    ) 
)

(define ProtectFoodGoal
    (EvaluationLink (stv 0.01 1.0)
        (PredicateNode "ProtectFoodGoal")
    )
)

; Actions
(define EatFoodAction
    (ExecutionLink (stv 0.3 1.0) 
        (GroundedSchemaNode "eat_food") 
    ) 
)

(define FindFoodAction
    (ExecutionLink (stv 0.3 1.0)
        (GroundedSchemaNode "find_food")
    )
)

(define ProtectFoodAction
    (ExecutionLink (stv 0.3 1.0)
        (GroundedSchemaNode "protect_food") 
    )
)

; OpenPsi Rules
(ImplicationLink (stv 0.80 1.0)
    (AndLink
        (AndLink     
            GetFoodGoal
            LunchTime
        )

        EatFoodAction
    )

    EnergyGoal
)

(ImplicationLink (stv 0.9 1.0)
    (AndLink
        (AndLink 
            FindFoodGoal
        )

        FindFoodAction
    )

    GetFoodGoal
)

(ImplicationLink (stv 0.7 1.0)
    (AndLink
        (AndLink
            ProtectFoodGoal
        )

        ProtectFoodAction
    )

    GetFoodGoal
)



