; $a->B 
; use InversionRule to create
; B->$a

(AverageLink (stv 1 1)
    (VariableNode "$a")
    (InheritanceLink (stv 1 1)
        (VariableNode "$a" (stv 1 1))
        (ConceptNode "B" (stv 1 1))
    )
)

# AverageLink means you can use that as a variable instead. if you skolemize things the right way then you don't have to do this. probably better to just make special terms for variables.


(InheritanceLink (stv 1 1)
    (ConceptNode "$pln_var_a" (stv 0.1 1))
    (ConceptNode "B" (stv 0.1 1))
)

