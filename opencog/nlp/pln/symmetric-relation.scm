;
; symmetric-relation.scm
;
; Quick hack/port of XML demo for "SymmetricRelation"
; Many of the truth values were messed with ... 

(define x
    (EvaluationLink (stv 0.8 0.6)
        (PredicateNode "friendOf")
        (ListLink 
            (ConceptNode "AAA" (stv 1 0.5))
            (ConceptNode "BBB" (stv 1 0.5))
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

(InheritanceLink (stv 0.999 0.999)
    (PredicateNode "friendOf")
    (ConceptNode "symmetricRelation")
)

; Define the meaning of a "symmetric relation"
; If R inherits from symmetricRelation and
; we have R(x,y) then deduce R(y,x)  
(ForallLink (stv 0.99 0.99)
    (ListLink (stv 1 0)
        (VariableNode "X007")
        (VariableNode "Y008")
        (VariableNode "R000")  ;; the relation
    )
    (ImplicationLink (stv 1 0)
        (AndLink (stv 1 0)
            (InheritanceLink (stv 1 0)
                (VariableNode "R000")   
                (ConceptNode "symmetricRelation")
            )
            (EvaluationLink (stv 1 0)
                (VariableNode "R000")
                (ListLink (stv 1 0)
                    (VariableNode "X007")
                    (VariableNode "Y008")
                )
            )
        )
        (EvaluationLink (stv 1 0)
            (VariableNode "R000")
            (ListLink (stv 1 0)
                (VariableNode "Y008")
                (VariableNode "X007")
            )
        )
    )
)

(define a
    (AndLink (stv 1 0.5)
        (InheritanceLink (stv 1 0.5)
            (VariableNode "R000")
            (ConceptNode "symmetricRelation")
        )
        (EvaluationLink (stv 1 0.5)
            (VariableNode "R000")
            (ListLink (stv 1 0)
                (VariableNode "X007")
                (VariableNode "Y008")
            )
        )
    )
)

(define b
    (EvaluationLink (stv 1 0.5)
        (VariableNode "R000")
        (ListLink (stv 1 0.5)
            (VariableNode "Y008")
            (VariableNode "X007")
        )
    )
)

(ForallLink (stv 0.99 0.99)
    (ListLink (stv 1 0)
        (VariableNode "X007")
        (VariableNode "Y008")
        (VariableNode "R000")
    )
    (ImplicationLink (stv 1 0.5)
        a b
    )
)
(pln-bc y 50)

(ConceptNode  "symmetricRelation" (stv 1 0.99))
(VariableNode  "R000" (stv 1 0.99) )


 (define d (cog-tv->alist (cog-tv y)))


(VariableNode  "X007" (stv 1 0.9) )
(VariableNode  "Y008" (stv 1 0.9) )
(VariableNode  "R000" (stv 1 0.9) )



