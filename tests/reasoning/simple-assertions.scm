; Define the concepts

(ConceptNode "Socrates" (stv .001 0.9))
(ConceptNode "Einstein" (stv .001 0.9))
(ConceptNode "Peirce" (stv .001 0.9))
(ConceptNode "man" (stv .01 0.9))
(ConceptNode "human" (stv .02 0.9))

; Define the instances of man

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "Socrates")
    (ConceptNode "man"))

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "Einstein")
    (ConceptNode "man"))

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "Peirce")
    (ConceptNode "man"))

; Define what man is part of

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "man")
    (ConceptNode "human"))

; Assign some additional memberships as well

(InheritanceLink (stv 0.9 0.9)
    (ConceptNode "Einstein")
    (ConceptNode "violin-players"))

; Pattern to match to check the output

(define find-humans
    (BindLink
        (VariableNode "$X")
        (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "human"))
        (ListLink
            (VariableNode "$X"))))
