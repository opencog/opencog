; The atoms below describe the space/environment which the agents are in.
; The world is a 3D one but information the 3rd dimenstion are not considered.

; Atoms describing the dimenstion of the 3D World
(EvaluationLink
    (PredicateNode "has")
    (ListLink
        (ConceptNode "3D_World")
        (ConceptNode "x-axis")
    )
)

(EvaluationLink
    (PredicateNode "has")
    (ListLink
        (ConceptNode "3D_World")
        (ConceptNode "y-axis")
    )
)

(EvaluationLink
    (PredicateNode "has")
    (ListLink
        (ConceptNode "3D_World")
        (ConceptNode "z-axis")
    )
)

(EvaluationLink
    (PredicateNode "has_length")
    (ListLink
        (ConceptNode "x-axis")
        (NumberNode 4)
    )
)

(EvaluationLink
    (PredicateNode "has_length")
    (ListLink
        (ConceptNode "y-axis")
        (NumberNode 9)
    )
)

(EvaluationLink
    (PredicateNode "has_length")
    (ListLink
        (ConceptNode "z-axis")
        (NumberNode 0)
    )
)

; Set of atoms in the 3D World
(MemberLink
    (AvatarNode "Bob")
    (ConceptNode "3D_World")
)

(MemberLink
    (HumanoidNode "me")
    (ConceptNode "3D_World")
)

(MemberLink
    (ObjectNode "house_15")
    (ConceptNode "3D_World")
)

(MemberLink
    (ObjectNode "house_18")
    (ConceptNode "3D_World")
)

(MemberLink
    (ObjectNode "random_object")
    (ConceptNode "3D_World")
)

; Properties of objects in the 3D World
;
(InheritanceLink
    (AvatarNode "$X")
    (ConceptNode "Agent")
)

(InheritanceLink
    (HumanoidNode "me")
    (ConceptNode "Agent")
)


