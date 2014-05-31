; An example for a contextual inference process taken
; from the Real-World-Reasoning book by Nils and Ben
; p. 212ff

; Axioms for music and accounting context

; 1) "In the context of music Alice frequently mentions
; Canadian place names."
(ContextLink (stv .5 .9)
    (ConceptNode "Music")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Alice")
            (ConceptNode "CanadianPlaceNames"))))



; 2) "In the context of music Bob frequently mentions
; Canadian place names."
(ContextLink (stv .5 .9)
    (ConceptNode "Music")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Bob")
            (ConceptNode "CanadianPlaceNames"))))

; 3) "In the context of music Clark does not frequently
; mention Canadian place names."
(ContextLink (stv .1 .9)
    (ConceptNode "Music")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Clark")
            (ConceptNode "CanadianPlaceNames"))))

; 4) "In the context of accounting Alice does not frequently
; mention Canadian place names."
(ContextLink (stv .1 .9)
    (ConceptNode "Accounting")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Alice")
            (ConceptNode "CanadianPlaceNames"))))

; 5) "In the context of accounting Bob frequently
; mentions Canadian place names."
(ContextLink (stv .5 .9)
    (ConceptNode "Accounting")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Bob")
            (ConceptNode "CanadianPlaceNames"))))


; 6) "In the context of accounting Clark frequently
; mentions Canadian place names."
(ContextLink (stv .6 .9)
    (ConceptNode "Accounting")
    (EvaluationLink
        (PredicateNode "mention")
        (ListLink
            (ConceptNode "Clark")
            (ConceptNode "CanadianPlaceNames"))))


; Non-context specific axioms

; 7) "Accounting is associated with money."
; Changed InheritanceLink to HebbianLink
(HebbianLink (stv .7 .9)
    (ConceptNode "Accounting")
    (ConceptNode "Money"))

; 8) "CanadianPlaces is associated with Canada."
(HebbianLink (stv .8 .9)
    (ConceptNode "CanadianPeople")
    (ConceptNode "CanadianPlacesNames"))

; 9) "If X frequently mentions Y then he/she is highly
; involved with Y."
(AverageLink (stv .9 .8)
    (ListLink
        (VariableNode "$X")
        (VariableNode "$Y"))
    (ImplicationLink
        (EvaluationLink
            (PredicadeNode "mention")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$Y")))
        (EvaluationLink
            (PredicadeNode "involved")
            (ListLink
                (VariableNode "$X")
                (VariableNode "$Y")))))
       

; 10) Non Canadian People involved with Canadian
; people in the context of money have a chance of
; being associated with log trafficking activities."
(AverageLink (stv .6 .8)
    (VariableNode "$X")
    (ImplicationLink
        (AndLink
            (SubsetLink
                (VariableNode "$X")
                (NotLink
                    (ConceptNode "CanadianPeople")))
            (ContextLink
                (ConceptNode "Money")
                (EvaluationLink
                    (PredicateNode "involved")
                    (ListLink
                        (VariableNode "$X")
                        (ConceptNode "CanadianPeople")))))
        (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "LogTrafficking"))))

; 11) "Clark is not Canadian."
(InheritanceLink (stv .9 .9)
    (ConceptNode "Clark")
    (NotLink
        (ConceptNode "CanadianPeople")))

; 12) It is also necessary to define the truth value
; of the following concepts:
(ConceptNode "Accounting" (stv .2 .9))
(ConceptNode "Music" (stv .3 .9))
(ConceptNode "CanadianPlaceNames" (stv .1 .8))
(ConceptNode "Money" (stv .2 .9))
(ConceptNode "CanadianPeople" (stv .25 .8))

; Question to answer:
; What is the chance of Clark being involved with log trafficking?
(define query
    (InheritanceLink (stv "$x" 0.0)
        (ConceptNode "Clark")
        (ConceptNode "LogTrafficking")))
