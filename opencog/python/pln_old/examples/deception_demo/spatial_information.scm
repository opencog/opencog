;; The atoms below describe the space/environment which the agents are in.
;; The world is a 3D one but information in the 3rd dimenstion are not considered.

; NOTE:
; * House_1 houses the Red_Battery.
; * House_2 is empty.

; Initial State.
(EvaluationLink (stv 1 1)
    (PredicateNode "GreaterThan")
    (ListLink
        (EvaluationLink (stv 1 1)
            (PredicateNode "Distance")
            (ListLink
                (ConceptNode "House_1")
                (ConceptNode "House_2")
            )
        )
        (EvaluationLink (stv 1 1)
            (PredicateNode "Distance")
            (ListLink
                (ConceptNode "Me")
                (ConceptNode "House_1")
            )
        )
    )
)

; Initial State.
(EvaluationLink (stv 1 1)
    (PredicateNode "EqualTo")
    (ListLink
        (EvaluationLink (stv 1 1)
            (PredicateNode "Distance")
            (ListLink
                (ConceptNode "Bob")
                (ConceptNode "House_1")
            )
        )
        (EvaluationLink (stv 1 1)
            (PredicateNode "Distance")
            (ListLink
                (ConceptNode "Me")
                (ConceptNode "House_1")
            )
        )
    )
)

; Initial State.
(EvaluationLink (stv 1 1)
    (PredicateNode "In")
    (ListLink
        (ConceptNode "Red_Battery")
        (ConceptNode "House_1")
    )
)

(EvaluationLink (stv 1 1)
    (PredicateNode "MoveTo")
    (ListLink
        (ConceptNode "Bob")
        (ConceptNode "House_2")
    )
)

