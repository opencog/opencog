; As described at http://wiki.opencog.org/w/Simple_Deception

; Expected output
;If I tell Bob there is a green battery behind House_15,
;then I will get to pick up the red battery
;(ImplicationLink ; should be predictiveimplicationlink
;    (EvaluationLink
;        (PredicateNode "tell")
;        (ListLink
;            (ConceptNode "me")
;            (ConceptNode "Bob")
;            (ExistsLink
;                (VariableNode "$X")
;                (AndLink ;SimultaneousAndLink
;                    (InheritanceLink 
;                        (VariableNode "$X")
;                        (ObjectNode "Battery")
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Behind")
;                        (ListLink
;                            (VariableNode "$X")
;                            (ConceptNode "House_15")
;                        )
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Near")
;                        (ListLink
;                            (VariableNode "$X")
;                            (ConceptNode "House_15")
;                        )
;                    )
;                )
;            )
;        )
;    )
;    (EvaluationLink
;        (PredicateNode "Pick_up")
;        (ListLink
;            (ConceptNode "me")
;            (ObjectNode "Red_Battery_18")
;        )
;    )
;)

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

(ImplicationLink
    (EvaluationLink
            (PredicateNode "believe")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
    )
    (EvaluationLink
        (PredicateNode "know")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
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

;Knowledge implies belief
(ImplicationLink
    (EvaluationLink
        (PredicateNode "know")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
    (EvaluationLink
        (PredicateNode "believe")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
)

;Belief approximately involves knowledge
(ImplicationLink (stv .5 .8)
    (EvaluationLink
        (PredicateNode "believe")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
    (EvaluationLink
        (PredicateNode "know")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$X")
        )
    )
)

;People often believe what they're told
(ImplicationLink (stv .8 .8)
    (EvaluationLink
        (PredicateNode "tell")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$X")
        )
    )
    (EvaluationLink
        (PredicateNode "believe")
        (ListLink
            (VariableNode "$B")
            (VariableNode "$X")
        )
    )
)

;If I tell Bob there is a battery behind  $x and near House_15, 
;he may believe there is a battery behind  $x and near House_15
(ImplicationLink
    (EvaluationLink
        (PredicateNode "tell")
        (ListLink
            (ConceptNode "me")
            (ConceptNode "Bob")
            (ExistsLink 
                (VariableNode"$X")
                (AndLink
                    (InheritanceLink
                        (VariableNode "$X")
                        (ConceptNode "Battery")
                    )
                    (EvaluationLink
                        (PredicateNode "Behind")
                        (ListLink
                            (VariableNode "$X")
                            (VariableNode "House_15")
                        )
                    )
                    (EvaluationLink
                        (PredicateNode "Near")
                        (ListLink
                            (VariableNode "$X")
                            (VariableNode "House_15")
                        )
                    )
                )
            )
        )
    )
    (EvaluationLink
        (PredicateNode "believe")
        (ListLink
            (ConceptNode "Bob")
            (ExistsLink
                (VariableNode "$X")
                (AndLink
                    (InheritanceLink
                        (VariableNode "$X")
                        (ConceptNode "Battery")
                    )
                    (EvaluationLink
                        (PredicateNode "Behind")
                        (ListLink
                            (VariableNode "$X")
                            (VariableNode "House_15")
                        )
                    )
                    (EvaluationLink
                        (PredicateNode "Near")
                        (ListLink
                            (VariableNode "$X")
                            (VariableNode "House_15")
                        )
                    )
                )
            )
        )
    )
)


;If an agent A wants some X, and believes that X is at location L,
;then A will likely move to L

(ImplicationLink (stv .7 .7)
    (AndLink
        (EvaluationLink
            (PredicateNode "Wants")
            (ListLink
                (VariableNode "$A")
                (VariableNode "$X")
            )
        )
        (EvaluationLink
            (PredicateNode "Believes")
            (ListLink
                (VariableNode "$A")
                (EvaluationLink
                    (PredicateNode "atLocation")
                    (ListLink
                        (VariableNode "$X")
                        (VariableNode "$L")
                    )
                )
            )
        )
    )
    (EvaluationLink
        (PredicateNode "MoveTo")
        (ListLink
            (VariableNode "$A")
            (VariableNode "$L")
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

; Modus Ponens for grounding $X
; Bob is likely to believe if we tell it something that is believable.
; if the can't see something then they are likely to beliveable.

