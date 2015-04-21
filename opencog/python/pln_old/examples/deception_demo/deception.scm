; As described at http://wiki.opencog.org/w/Simple_Deception
; with few simplifications.

(InheritanceLink (stv 1.000000 1.000000)
    (ConceptNode "Me")
    (ConceptNode "Agent")
)

(InheritanceLink (stv 1.000000 1.000000)
    (ConceptNode "Bob")
    (ConceptNode "Agent")
)

;; The_Battery is a battery that is the imaginary/Not_real battery that doesn't
;; in the environment exist.
;(InheritanceLink (stv .9 .1)
;    (ConceptNode "The_Battery")
;    (ConceptNode "Battery")
;)

;(InheritanceLink (stv 1 1)
;    (ConceptNode "The_Battery")
;    (ConceptNode "Not_real")
;)

;(EvaluationLink (stv 1 1)
;    (PredicateNode "Knows")
;    (ListLink
;        (ConceptNode "Me")
;        (InheritanceLink (stv 1 1)
;            (ConceptNode "The_Battery")
;            (ConceptNode "Not_real")
;        )
;    )
;)

; Agents want battery.
(ImplicationLink (stv 1 1) ; working
    (InheritanceLink
        (VariableNode "$X@4")
        (ConceptNode "Agent")
    )
    (EvaluationLink
        (PredicateNode "Wants")
        (ListLink
            (VariableNode "$X@4")
            (ConceptNode "Battery")
        )
    )
)

;;Knowledge implies belief
;(ImplicationLink (stv 1 1) ; working
;    (EvaluationLink
;        (PredicateNode "Knows")
;        (ListLink
;            (VariableNode "$A@5")
;            (VariableNode "$X@5")
;        )
;    )
;    (EvaluationLink
;        (PredicateNode "Believes")
;        (ListLink
;            (VariableNode "$A@5")
;            (VariableNode "$X@5")
;        )
;    )
;)

;;Belief approximately involves knowledge
;(ImplicationLink (stv .5 .8) ; working
;    (EvaluationLink
;        (PredicateNode "Believes")
;        (ListLink
;            (VariableNode "$A@6")
;            (VariableNode "$X@6")
;        )
;    )
;    (EvaluationLink
;        (PredicateNode "Knows")
;        (ListLink
;            (VariableNode "$A@6")
;            (VariableNode "$X@6")
;        )
;    )
;)

;Agents often believe what they're told
(ImplicationLink (stv .8 .8) ;working
    (EvaluationLink
        (PredicateNode "Tell")
        (ListLink
            (VariableNode "$A@7")
            (VariableNode "$B@7")
            (VariableNode "$X@7")
        )
    )
    (EvaluationLink
        (PredicateNode "Believes")
        (ListLink
            (VariableNode "$B@7")
            (VariableNode "$X@7")
        )
    )
)

; If an agent A wants some X, and believes that X is In L, then A will
; likely move to L (This is a behavior)
(ImplicationLink (stv .7 .7)
    (AndLink (stv 1 1)
        (EvaluationLink
            (PredicateNode "Wants")
            (ListLink
                (VariableNode "$A@8")
                (VariableNode "$X@8")
            )
        )
        (EvaluationLink
            (PredicateNode "Believes")
            (ListLink
                (VariableNode "$A@8")
                (EvaluationLink
                    (PredicateNode "In")
                    (ListLink
                        (VariableNode "$X@8")
                        (VariableNode "$L@8")
                    )
                )
            )
        )
    )
    (EvaluationLink
        (PredicateNode "MoveTo")
        (ListLink
            (VariableNode "$A@8")
            (VariableNode "$L@8")
        )
    )
)

; Whatever be the relation between two Concepts such that $X $Wants $Battery and
; there is $Y which is $Battery then $X $Wants $Y too. (This is crude in the sense
; that the truth values isn't revised debending on the stv of the inheritance and
; the stv of the inital EvaluationLink)
(ImplicationLink (stv 1 1) ;working
    (AndLink (stv 1 1)
        (EvaluationLink
            (VariableNode "$Wants")
            (ListLink
                (VariableNode "$X@9")
                (VariableNode "$Battery@9")
            )
        )
        (InheritanceLink
            (VariableNode "$Y@9")
            (VariableNode "$Battery@9")
        )
    )
    (EvaluationLink
        (VariableNode "$Wants")
        (ListLink
            (VariableNode "$X@9")
            (VariableNode "$Y@9")
        )
    )
)

