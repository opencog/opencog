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

;Belief implies Knowledge
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

;Agents often believe what they're told
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
; Possibly an intermediate output
;(ImplicationLink
;    (EvaluationLink
;        (PredicateNode "tell")
;        (ListLink
;            (ConceptNode "me")
;            (ConceptNode "Bob")
;            (ExistsLink 
;                (VariableNode"$X")
;                (AndLink
;                    (InheritanceLink
;                        (VariableNode "$X")
;                        (ConceptNode "Battery")
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Behind")
;                        (ListLink
;                            (VariableNode "$X")
;                            (VariableNode "House_15")
;                        )
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Near")
;                        (ListLink
;                            (VariableNode "$X")
;                            (VariableNode "House_15")
;                        )
;                    )
;                )
;            )
;        )
;    )
;    (EvaluationLink
;        (PredicateNode "believe")
;        (ListLink
;            (ConceptNode "Bob")
;            (ExistsLink
;                (VariableNode "$X")
;                (AndLink
;                    (InheritanceLink
;                        (VariableNode "$X")
;                        (ConceptNode "Battery")
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Behind")
;                        (ListLink
;                            (VariableNode "$X")
;                            (VariableNode "House_15")
;                        )
;                    )
;                    (EvaluationLink
;                        (PredicateNode "Near")
;                        (ListLink
;                            (VariableNode "$X")
;                            (VariableNode "House_15")
;                        )
;                    )
;                )
;            )
;        )
;    )
;)

;If an agent A wants some X, and believes that X is at location L, then A will
; likely move to L
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

; Agents want battery.
(ImplicationLink
        (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "Agent")
        )
        (EvaluationLink
            (PredicateNode "want")
            (ListLink
                (VariableNode "$X")
                (ObjectNode "Battery")
            )
        )
    )
)

; The spatial information that is going to be infered from the spatial reasoning
; will give the following output.
(NotLink
    (EvaluationLink
        (PredicateNode "Near")
        (ListLink
            (ConceptNode "Bob")
            (ConceptNode "Red_Battery_15")
        )
    )
)

