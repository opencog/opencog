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


; The pattern specified here is buggy: its an invalid pattern
; because the variable $x never appears unquoted in the pattern.
; In fact, the pattern has no variables in it at all, since any
; quoted variable is not a variable, but is a constant.
(define query_rule
    (BindLink (stv 1 1)
        (ListLink (stv 1 1)
            (VariableNode "$x")
        )
        (ImplicationLink (stv 1 1)
            (InheritanceLink
                (QuoteLink
                    (VariableNode "$x")
                )
                (ConceptNode "criminal")
            )
            (InheritanceLink
                (VariableNode "$x")
                (ConceptNode "criminal")
            )
        )
    )
)
