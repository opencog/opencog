; generated from fetchdemo5.xml then cleaned at hand

; concepts
(define ball (ConceptNode "ball"))
(define teacher (ConceptNode "teacher"))

; actions
(define walktowards (SchemaNode "walktowards"))
(define give (SchemaNode "give"))
(define hold (SchemaNode "hold"))
(define lift (SchemaNode "lift"))

; variables
(define Vobj (VariableNode "$obj"))
(define V10000 (VariableNode "$10000"))
(define V1 (VariableNode "$1"))
(define V2 (VariableNode "$2"))

; predicates
(define Reward (PredicateNode "+++"))
(define teacher_say (PredicateNode "teacher_say"))
(define just_done (PredicateNode "just_done"))
(define near (PredicateNode "near"))
(define do (PredicateNode "do"))
(define can_do (PredicateNode "can_do"))

; words
(define fetch (WordNode "fetch"))

; lists (predicate's arguments)
(define L_ball (ListLink ball))
(define L_teacher (ListLink teacher))
(define L_Vobj (ListLink Vobj))
(define L_V1 (ListLink V1))
(define L_V2 (ListLink V2))
(define L_ball_teacher (ListLink ball teacher))

; define evaluations
(define walktowards_ball (EvaluationLink walktowards L_ball))
(define walktowards_teacher (EvaluationLink walktowards L_teacher))
(define walktowards_V2 (EvaluationLink walktowards L_V2))
(define give_ball_teacher (EvaluationLink give L_ball_teacher))
(define hold_ball (EvaluationLink hold L_ball))
(define lift_Vobj (EvaluationLink lift L_Vobj))
(define lift_ball (EvaluationLink lift L_ball))

(ForAllLink (stv 1 0.80000001) L_Vobj
            (ImplicationLink (EvaluationLink near
                                             L_Vobj)
                             (EvaluationLink can_do
                                             (ListLink lift_Vobj))))

(ForAllLink (stv 1 0.99900001) (ListLink V10000)
            (EvaluationLink do
                            (ListLink V10000)))

(ImplicationLink (stv 1 0.99000001) (AndLink (EvaluationLink just_done
                                                             (ListLink hold_ball))
                                             (EvaluationLink near
                                                             L_teacher))
                 (EvaluationLink can_do
                                 (ListLink give_ball_teacher)))

(ImplicationLink (stv 1 0.99000001) (EvaluationLink just_done
                                                    (ListLink lift_ball))
                 (EvaluationLink can_do
                                 (ListLink hold_ball)))

(ImplicationLink (stv 1 0.80000001) (AndLink (EvaluationLink (stv 0.99000001 0.99000001) teacher_say
                                                             (ListLink fetch))
                                             (EvaluationLink just_done
                                                             (ListLink give_ball_teacher)))
                 (EvaluationLink Reward))

(ImplicationLink (stv 1 0.99900001) (AndLink (EvaluationLink just_done
                                                             (ListLink walktowards_V2)))
                 (EvaluationLink near
                                 L_V2))

(ImplicationLink (stv 1 0.99900001) (AndLink (EvaluationLink do
                                                             L_V1)
                                             (EvaluationLink can_do
                                                             L_V1))
                 (EvaluationLink just_done
                                 L_V1))
(EvaluationLink (stv 1 0.99000001) can_do
                (ListLink walktowards_teacher))

(EvaluationLink (stv 1 0.99000001) can_do
                (ListLink walktowards_ball))

(EvaluationLink (stv 0.99000001 0.99000001) teacher_say
                (ListLink fetch))

