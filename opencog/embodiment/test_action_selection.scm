(define p 
    (list 1 2 3 4 5 6 7 8 9)
)

(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "is_edible") 
    (ListLink
        (ObjectNode "apple") 
    )
)

(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "is_edible") 
    (ListLink
        (ObjectNode "pear") 
    )
)

(EvaluationLink (stv 0 0)
    (PredicateNode "is_edible") 
    (ListLink
        (AvatarNode "Troy") 
    )
)

(EvaluationLink (stv 0 0)
    (PredicateNode "is_edible") 
    (ListLink
        (ObjectNode "cup") 
    )
)

(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "near") 
    (ListLink
        (ObjectNode "apple") 
    )
)

(EvaluationLink (stv 0.0 1.0)
    (PredicateNode "near") 
    (ListLink
        (ObjectNode "pear") 
    )
)

(EvaluationLink (stv 1.0 1.0)
    (PredicateNode "near") 
    (ListLink
        (AvatarNode "Troy") 
    )
)

(EvaluationLink (stv 0 0)
    (PredicateNode "near") 
    (ListLink
        (ObjectNode "cup") 
    )
)

(define GetFoodGoal
    (AndLink 
        (EvaluationLink
            (PredicateNode "is_edible") 
            (ListLink
                (VariableNode "$var_food")
            )    
        )

        (EvaluationLink
            (PredicateNode "near") 
            (ListLink
                (VariableNode "$var_food") 
            )
        )
    )
)

(define EnergyDemandGoal
    (EvaluationLink
        (PredicateNode "EnergyDemandGoal") 
        (VariableNode "$var_food")
    )
)

(define EatAction
    (ExecutionLink
        (GroundedSchemaNode "eat") 
        (ListLink
            (VariableNode "$var_food") 
        )
    )
)

(define RandomSearchAction
    (ExecutionLink
        (GroundedSchemaNode "random_search") 
        (ListLink)
    ) 
)

(define GotoFoodAction
     (ExecutionLink
        (GroundedSchemaNode "goto_object") 
        (ListLink
            (VariableNode "$var_food") 
        )
    )

)

(define EnergyDemandRule
    (ImplicationLink
        (AndLink
            GetFoodGoal
            (AndLink
                GotoFoodAction
                EatAction
            )
        )
        EnergyDemandGoal
    )
)

(define GetFoodRule
    (ImplicationLink
        RandomSearchAction
        GetFoodGoal
    )
)

(ForAllLink
    (ListLink
        (VariableNode "$var_food")
    )
    GetFoodRule
)

(ForAllLink
    (ListLink
        (VariableNode "$var_food") 
    ) 
    EnergyDemandRule
)

(random_select p)

(cog-link? EnergyDemandGoal)
(cog-node? EnergyDemandGoal)    

(cog-bind (find_psi_rule EnergyDemandGoal) )
(cog-bind (find_psi_rule GetFoodGoal) )

(cog-bind (find_psi_action (VariableNode "$var_food") GetFoodGoal EatAction ) )

(split_context_action GetFoodGoal)
(split_context_action EatAction)

(split_context_action (AndLink GetFoodGoal EatAction) )
     
(cal_truth_value GetFoodGoal)
(cal_truth_value EatAction)

(is_psi_context_true GetFoodRule)
(is_psi_context_true EnergyDemandRule)

(display "========================================================") (make_psi_plan GetFoodGoal)

(display "========================================================") (make_psi_plan EnergyDemandGoal)

