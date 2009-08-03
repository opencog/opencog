;
; symmetric-relation.scm
; quick hack/port of XML demo for "SymmetricRelation"

(define x
    (EvaluationLink (stv 0.8 0.6)
        (PredicateNode "friendOf")
        (ListLink 
            (ConceptNode "AAA")
            (ConceptNode "BBB")
        )
    )
)

(define y
    (EvaluationLink 
        (PredicateNode "friendOf")
        (ListLink 
            (ConceptNode "BBB")
            (ConceptNode "AAA")
        )
    )
)

(InheritanceLink (stv 0.999000 0.999000)
    (PredicateNode "friendOf")
    (ConceptNode "symmetricRelation")
)

(ForallLink (stv 0.990000 0.990000)
    (ListLink (stv 1.000000 0.000000)
        (VariableNode "X007")
        (VariableNode "X008")
        (VariableNode "R000")
    )
    (ImplicationLink (stv 1.000000 0.000000)
        (AndLink (stv 1.000000 0.000000)
            (InheritanceLink (stv 1.000000 0.000000)
                (VariableNode "R000")
                (ConceptNode "symmetricRelation")
            )
            (EvaluationLink (stv 1.000000 0.000000)
                (VariableNode "R000")
                (ListLink (stv 1.000000 0.000000)
                    (VariableNode "X007")
                    (VariableNode "X008")
                )
            )
        )
        (EvaluationLink (stv 1.000000 0.000000)
            (VariableNode "R000")
            (ListLink (stv 1.000000 0.000000)
                (VariableNode "X008")
                (VariableNode "X007")
            )
        )
    )
)

(define a
        (AndLink (stv 1.000000 0.000000)
            (InheritanceLink (stv 1.000000 0.000000)
                (VariableNode "R000")
                (ConceptNode "symmetricRelation")
            )
            (EvaluationLink (stv 1.000000 0.000000)
                (VariableNode "R000")
                (ListLink (stv 1.000000 0.000000)
                    (VariableNode "X007")
                    (VariableNode "X008")
                )
            )
        )
)
(define b
        (EvaluationLink (stv 1.000000 0.000000)
            (VariableNode "R000")
            (ListLink (stv 1.000000 0.000000)
                (VariableNode "X008")
                (VariableNode "X007")
            )
        )
)

(ForallLink (stv 0.990000 0.990000)
    (ListLink (stv 1.000000 0.000000)
        (VariableNode "X007")
        (VariableNode "X008")
        (VariableNode "R000")
    )
    (ImplicationLink (stv 1.000000 0.000000)
        a b
    )
)
(pln-bc y 100)

 (define a (cog-tv->alist (cog-tv y)))


(VariableNode  "X007" (stv 1.000000 0.000000) )
(VariableNode   "X008" (stv 1.000000 0.000000) )
(VariableNode  "R000" (stv 1.000000 0.000000) )
(ConceptNode  "symmetricRelation" (stv 1.000000 0.000000))



