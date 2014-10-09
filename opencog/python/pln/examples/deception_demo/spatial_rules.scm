; The following are 


;If an agent can see two objects X and Y,
;it probably can see the spatial relationships
;between them
(ImplicationLink
    (AndLink
        (EvaluationLink
            (PredicateNode "see")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
        )
        (EvaluationLink
            (PredicateNode "see")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$y")
            )
        )
        (EvaluationLink
            (PredicateNode "$R")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$y")
            )
        )
        (InheritanceLink 
            (VariableNode "$R")
            (ObjectNode "SpatialRelation")
        )
    )
    (EvaluationLink
            (PredicateNode "know")
            (ListLink
                (VariableNode "$A")
                (EvaluationLink
                    (PredicateNode "$R")
                    (ListLink
                        (VariableNode "$X")
                        (VariableNode "$y")
                    )
                )
            )
    )
)

;A conjunction of spatial predicates is a spatial predicate
(ImplicationLink
    (AndLink
        (InheritanceLink
            (VariableNode "$R")
            (ConceptNode "SpatialPredicate")
        )
        (InheritanceLink
            (VariableNode "$S")
            (ConceptNode "SpatialPredicate")
        )
    )
    (InheritanceLink
        (AndLink
            (VariableNode "$R")
            (VariableNode "$S")
        )
        (ConceptNode "SpatialPredicate")
    )
)

(ImplicationLink
    (AndLink
        (InheritanceLink
            (VariableNode "$R")
            (ConceptNode "SpatialRelation")
        )
        (EvaluationLink
            (VariableNode "$R")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$B")
            )
        )
    )
    (EvaluationLink
        (PredicateNode "atLocation")
        (ListLink
            (VariableNode "$B")
            (SatisfyingSetLink
                (VariableNode "$X")
                (EvaluationLink
                    (VariableNode "$R")
                    (ListLink
                        (VariableNode "$A")
                        (VariableNode "$X")
                    )
                )
            )
        )
    )
)

;If an agent can see an object X, it probably
;can see X's visual properties (the definition
;of "visual property"
(ImplicationLink
    (AndLink
        (EvaluationLink
            (PredicateNode "see")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
        )
        (InheritanceLink
            (VariableNode "$X")
            (VariableNode "$P")
        )
        (InheritanceLink
            (VariableNode "$P")
            (ConceptNode "VisualProperty")
        )
        (EvaluationLink (stv 1 .8)
            (PredicateNode "know")
            (ListLink
                (VariableNode "$A")
                (InheritanceLink
                    (VariableNode "$X")
                    (VariableNode "$P")
                )
            )
        )
    )
)

(InheritanceLink
    (ConceptNode "Color")
    (ConceptNode "VisualProperty")
)

(InheritanceLink
    (ConceptNode "Red")
    (ConceptNode "Color")
)

;If X is near Y, but X is not near Z, then Y is not near Z
(ImplicationLink (stv .8 .7)
    (AndLink
        (EvaluationLink
            (PredicateNode "Near")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$Y")
            )
        )
        (NotLink
            (EvaluationLink
                (PredicateNode "Near")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Z")
                )
            )
        )
    )
    (NotLink
        (EvaluationLink
            (PredicateNode "Near")
            (ListLink
                (VariableNode "$Y")
                (VariableNode "$Z")
            )
        )
    )
)


;If Bob is near House_15, and House_15 is not near Red_Battery_18,
;then Bob is not near Red_Battery_18
(ImplicationLink
    (AndLink
        (EvaluationLink
            (PredicateNode "Near")
            (ListLink
                (ConceptNode "Bob")
                (ConceptNode "House_15")
            )
        )
        (NotLink
             (EvaluationLink
                (PredicateNode "Near")
                (ListLink
                    (ConceptNode "House_15")
                    (ConceptNode "Red_Battery_18")
                )
            )
        )
    )
    (NotLink
        (EvaluationLink
            (PredicateNode "Near")
            (ListLink
                (ConceptNode "Bob")
                (ConceptNode "Red_Battery_15")
            )
        )
    )
)

;If X is near Y, but X is not near Z, then X is further from Z than from Y
(ImplicationLink (stv .8 .7)
    (AndLink
        (EvaluationLink
            (PredicateNode "Near")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$Y")
            )
        )
        (NotLink
            (EvaluationLink
                (PredicateNode "Near")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Z")
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
                    (VariableNode "$X")
                    (VariableNode "$Z")
                )
            )
            (EvaluationLink
                (PredicateNode "Distance")
                (ListLink
                    (VariableNode "$X")
                    (VariableNode "$Y")
                )
            )
        )
    )
)

;Spatial predicates specify locations.
(ImplicationLink
    (AndLink
        (InheritanceLink
            (VariableNode "$R")
            (ConceptNode "SpatialRelation")
        )
        (EvaluationLink
            (VariableNode "$R")
            (ListLink 
                (VariableNode "$A")
                (VariableNode "$B")
            )
        )
    )
    (EvaluationLink
        (PredicateNode "atLocation")
        (ListLink
            (VariableNode "$A")
            (SatisfyingSetLink
                (VariableNode "$X")
                (EvaluationLink
                    (VariableNode "$R")
                    (ListLink
                        (VariableNode "$X")
                        (VariableNode "$B")
                    )
                )
            )
        )
    )
)

;An agent must be near something to pick it up
(ImplicationLink
    (EvaluationLink
        (PredicateNode "PickUp")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
    (EvaluationLink
        (PredicateNode "Near")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
)


;If A and B both want some X, but
;B is closer to X, then B is more likely to get it
(ImplicationLink (stv .7 .3)
    (AndLink
        (EvaluationLink
            (PredicateNode "want")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
        )
        (EvaluationLink
            (PredicateNode "want")
            (ListLink
                (VariableNode "$B")
                (VariableNode "$X")
            )
        )
        (EvaluationLink
            (PredicateNode "GreaterThan")
            (ListLink
                (EvaluationLink
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$A")
                        (VariableNode "$X")
                    )
                )
                (EvaluationLink
                    (PredicateNode "Distance")
                    (ListLink
                        (VariableNode "$B")
                        (VariableNode "$X")
                    )
                )
            )
        )
    )
    (AndLink
        (EvaluationLink
            (PredicateNode "PickUp")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
        )
        (NotLink
            (EvaluationLink
                (PredicateNode "PickUp")
                (ListLink
                    (VariableNode "$B")
                    (VariableNode "$X")
                )
            )
        )
    )
)
