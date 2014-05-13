; Socrates, Einstein, and Peirce are all men.
(ConceptNode "Socrates" (stv .001 1))
(ConceptNode "Einstein" (stv .001 1))
(ConceptNode "Peirce" (stv .001 1))
(ConceptNode "man" (stv .01 1))

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "Socrates")
    (ConceptNode "man"))

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "Einstein")
    (ConceptNode "man"))

(InheritanceLink (stv 1.0 1.0)
    (ConceptNode "Peirce")
    (ConceptNode "man"))

; Pattern for the pattern matcher to match, to find them.
(define find-man
    (BindLink
        (ListLink
            (VariableNode "$X"))
        (ImplicationLink
            (InheritanceLink
                (VariableNode "$X")
                (ConceptNode "man"))
            (VariableNode "$X"))))
