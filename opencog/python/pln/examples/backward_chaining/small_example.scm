; Reduced backward-chaining example

(ImplicationLink
    (InheritanceLink
        (VariableNode "$x")
        (ConceptNode "American"))
    (InheritanceLink
        (VariableNode "$x")
        (ConceptNode "criminal")))

(InheritanceLink
    (ConceptNode "West")
    (ConceptNode "American"))

(define isCriminal
    (InheritanceLink
        (VariableNode "$isCriminal")
        (ConceptNode "criminal")))
