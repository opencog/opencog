; A car is not a boat.
; Some boats are water bicycles.
; |- ?
; A. No boat is a water bicycle.
; B. Some water bicycles are no cars.
; C. No boat is a car.
; D. Some cars are no water bicycles.

(EvaluationLink (PredicateNode "inputs")
    (ListLink
        ; "A car is not a boat"
        (InheritanceLink (stv 0.99 0.99)
            (ConceptNode "boat")
            (NotLink
                (ConceptNode "car")
            )
        )
        ; "Some boats are water bicycles"
        (EvaluationLink (stv 0.99 0.99)
            (PredicateNode "some")
            (ListLink
                (InheritanceLink
                    (ConceptNode "boat")
                    (ConceptNode "water_bicycle")
                )
            )
        )
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "ModusPonensRule:ImplicationLink")
	)
)

(EvaluationLink (PredicateNode "desired_outputs")
    (ListLink
        ; "Some water bicycles are no cars"
        (EvaluationLink (stv 0.99 0.99)
            (PredicateNode "some")
            (ListLink
                (InheritanceLink
                    (ConceptNode "water_bicycle")
                    (NotLink
                        (ConceptNode "car")
                    )
                )
            )
        )
        ; "No boat is a car"
        (InheritanceLink (stv 0.99 0.99)
            (ConceptNode "boat")
            (NotLink
                (ConceptNode "car")
            )
        )
        ; "Some cars are no water bicycles"
        (EvaluationLink (stv 0.99 0.99)
            (PredicateNode "some")
            (ListLink
                (InheritanceLink
                    (ConceptNode "car")
                    (NotLink
                        (ConceptNode "water_bicycles")
                    )
                )
            )
        )
    )
)

(EvaluationLink (PredicateNode "undesired_outputs")
    (ListLink
        ; "No boat is a water bicycle"
        (InheritanceLink (stv 0.99 0.99)
            (ConceptNode "boat")
            (NotLink
                (ConceptNode "water_bicycle")
            )
        )
    )
)
