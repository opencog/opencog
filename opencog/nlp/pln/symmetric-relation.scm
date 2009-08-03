;
; symmetric-relation.scm
;
; Quick hack/port of XML demo for "SymmetricRelation"
; Many of the truth values were messed with ... 

; The "fact" that we start with
(define x
    (EvaluationLink (stv 0.8 0.4)
        (PredicateNode "friendOf" (stv 1 0))
        (ListLink 
            (ConceptNode "Amir" (stv 0.00001 0.999))
            (ConceptNode "Britney" (stv 0.00001 0.999))
        )
    )
)

; A conclusion that we'd like to either prove, or disprove
(define y
    (EvaluationLink 
        (PredicateNode "friendOf")
        (ListLink 
            (ConceptNode "Britney")
            (ConceptNode "Amir")
        )
    )
)

; Define friendOf as a symmetric relation
(InheritanceLink (stv 0.999 0.999)
    (PredicateNode "friendOf")
    (ConceptNode "symmetricRelation" (stv 1 0))
)

; Define the meaning of a "symmetric relation"
; If R inherits from symmetricRelation and
; we have R(x,y) then deduce R(y,x)  
(ForallLink (stv 0.99 0.99)
    (ListLink (stv 1 0)
        (VariableNode "X007" (stv 1 0))
        (VariableNode "Y008" (stv 1 0))
        (VariableNode "R000" (stv 1 0))  ;; the relation
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

(pln-bc y 250)

(ConceptNode  "symmetricRelation" (stv 1 0.99))
(VariableNode  "R000" (stv 1 0.99) )


 (define d (cog-tv->alist (cog-tv y)))

(define p (ConceptNode  "___PLN___"))


; Change confidence on the varaibles
(VariableNode  "X007" (stv 1 0.9) )
(VariableNode  "Y008" (stv 1 0.9) )
(VariableNode  "R000" (stv 1 0.9) )


; Alternative way of writing/defining the symmetric relation,
; completely the same as above.
(define a
    (AndLink 
        (InheritanceLink 
            (VariableNode "R000")
            (ConceptNode "symmetricRelation")
        )
        (EvaluationLink 
            (VariableNode "R000")
            (ListLink (stv 1 0)
                (VariableNode "X007")
                (VariableNode "Y008")
            )
        )
    )
)

(define b
    (EvaluationLink 
        (VariableNode "R000")
        (ListLink 
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
    (ImplicationLink (stv 1 0)
        a b
    )
)

