; Example taken from "Artifical Intelligence - A Modern Approach"

; "The law says that it is a crime for an American to sell weapons to hostile
; nations. The country Nono, an enemy of America, has some missiles, and all
; of its missiles were sold to it by ColonelWest, who is American."

; "... it is a crime for an American to sell weapons to hostile nations":
; American(x) ∧ Weapon(y) ∧ Sells(x, y, z) ∧ Hostile(z) ⇒ Criminal (x).
(ImplicationLink (stv .99 .99)
    (AndLink
        (InheritanceLink
            (VariableNode "$x")
            (ConceptNode "American"))
        (InheritanceLink
            (VariableNode "$y")
            (ConceptNode "weapon"))
        (EvaluationLink
            (PredicateNode "sell")
            (ListLink
                (VariableNode "$x")
                (VariableNode "$y")
                (VariableNode "$z")))
        (InheritanceLink
            (VariableNode "$z")
            (ConceptNode "hostile")))
    (InheritanceLink
        (VariableNode "$x")
        (ConceptNode "criminal")))

; "Nono ... has some missiles."
; ∃x Owns(Nono, x) ∧ Missile(x) is transformed by Existential Instantiation to
; Owns(Nono,M1), Missile(M1)
(AndLink (stv .99 .99)
    (InheritanceLink (stv .99 .99)
        (ConceptNode "missile@123")
        (ConceptNode "missile"))
    (EvaluationLink (stv .99 .99)
        (PredicateNode "own")
        (ListLink
            (ConceptNode "Nono")
            (ConceptNode "missile@123"))))

; "All of its missiles were sold to it by Colonel West":
; Missile(x) ∧ Owns(Nono, x) ⇒ Sells(West, x, Nono) .
(ImplicationLink (stv .99 .99)
    (AndLink
        (InheritanceLink
            (VariableNode "$a")
            (ConceptNode "missile"))
        (EvaluationLink
            (PredicateNode "own")
            (ListLink
                (ConceptNode "Nono")
                (VariableNode "$a"))))
    (EvaluationLink
        (PredicateNode "sell")
        (ListLink
            (ConceptNode "West")
            (VariableNode "$a")
            (ConceptNode "Nono"))))

; Missiles are weapons: Missile(x) ⇒ Weapon(x)
(InheritanceLink (stv .99 .99)
    (ConceptNode "missile")
    (ConceptNode "weapon"))

; An enemy of America is "hostile": Enemy(x,America) ⇒ Hostile(x) .
(ImplicationLink (stv .99 .99)
    (EvaluationLink
        (PredicateNode "enemy_of")
        (ListLink
            (VariableNode "$b")
            (ConceptNode "America")))
    (InheritanceLink
        (VariableNode "$b")
        (ConceptNode "hostile")))

; "West, who is American ...": American(West).
(InheritanceLink (stv .99 .99)
    (ConceptNode "West")
    (ConceptNode "American"))

; "The country Nono, an enemy of America ...": Enemy(Nono,America).
(EvaluationLink (stv .99 .99)
    (PredicateNode "enemy_of")
    (ListLink
        (ConceptNode "Nono")
        (ConceptNode "America")))

; example query: "Who is a criminal?"; query is checked in backward_agent
;(define isCriminal
;    (InheritanceLink
;        (VariableNode "$isCriminal")
;        (ConceptNode "criminal")))
