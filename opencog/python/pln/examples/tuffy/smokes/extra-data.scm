(define neighbors (PredicateNode "neighbors"))
(define knows (PredicateNode "knows"))
(define hasMet (PredicateNode "HasMet"))
(define sick (PredicateNode "sick"))
(define tired (PredicateNode "tired"))
(define intelligent (ConceptNode "intelligent"))
(define successful (ConceptNode "successful"))
(define happy (ConceptNode "happy"))

; Anna Bob Edward Frank Gary Helen
; Cancer: Bob, Frank, Anna, Edward

; A and B are neighbors.
(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Anna
        Frank))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Edward
        Bob))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Edward
        Gary))

(EvaluationLink (stv 1.0 1.0)
    neighbors
    (ListLink
        Gary
        Helen))

; Neighbors know each other.
(ImplicationLink (stv 0.9 1.0)
    (EvaluationLink
        neighbors
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (EvaluationLink
        knows
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y"))))

; People who know each other have met.
(ImplicationLink (stv 0.9 1.0)
    (EvaluationLink
        knows
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y")))
    (EvaluationLink
        hasMet
        (ListLink
            (VariableNode "$X")
            (VariableNode "$Y"))))

; A is intelligent.
(InheritanceLink (stv 0.9 1.0)
    Anna
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Frank
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Gary
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Bob
    intelligent)

(InheritanceLink (stv 0.9 1.0)
    Helen
    intelligent)

; Intelligent people are successful.
(ImplicationLink (stv 0.9 1.0)
    (InheritanceLink
        (VariableNode "$X")
        intelligent)
    (InheritanceLink
        (VariableNode "$X")
        successful))

; Successful people are happy.
(ImplicationLink (stv 0.9 1.0)
    (InheritanceLink
        (VariableNode "$X")
        successful)
    (InheritanceLink
        (VariableNode "$X")
        happy))

; People who have cancer are sick.
(ImplicationLink (stv 1.0 1.0)
    (EvaluationLink
        cancer
        (ListLink
            (VariableNode "$X")))
    (EvaluationLink
        sick
        (ListLink
            (VariableNode "$X"))))

; People who are sick are tired.
(ImplicationLink (stv 1.0 1.0)
    (EvaluationLink
        sick
        (ListLink
            (VariableNode "$X")))
    (EvaluationLink
        tired
        (ListLink
            (VariableNode "$X"))))
