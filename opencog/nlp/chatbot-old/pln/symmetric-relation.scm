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

(pln-bc y 100)


(define (prt-atom h) (display h) #f)

(cog-map-type prt-atom 'ConceptNode)

(define prt-all
   (map (lambda (x) (cog-map-type prt-atom x)) (cog-get-types))
)

(define (cnt-all)
	(define cnt 0)
	(define (ink a) (set! cnt (+ cnt 1)) #f)
	(map (lambda (x) (cog-map-type ink x)) (cog-get-types))
	cnt
)

(cog-map-type (lambda (x) (cog-extract-recursive! x) #f) 'FWVariableNode)
(cog-map-type (lambda (x) (cog-extract-recursive! x) #f) 'OrderedLink)
(cog-map-type prt-atom 'FWVariableNode)

(cog-extract-recursive! (ConceptNode "___PLN___"))



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


(ImplicationLink 
(mtv (stv 1 0)(vh "CONTEXTUAL" 96).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 188).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 207).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 134).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 196).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 162).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 109).(stv 0.99000001 0.99000001)(vh "CONTEXTUAL" 203).(stv 0.99000001 0.99000001)) 

(AndLink (mtv (stv 1 0)(vh "CONTEXTUAL" 96).(stv 0.7992 0.3996)) 
   (InheritanceLink (stv 0.99900001 0.99900001) 
      (PredicateNode "friendOf" (stv 1 0))
      (ConceptNode "symmetricRelation" (stv 1 0)))
   (EvaluationLink (stv 0.80000001 0.40000001) 
       (PredicateNode "friendOf" (stv 1 0))
       (ListLink 
          (ConceptNode "Amir" (stv 9.9999997e-06 0.99900001))
          (ConceptNode "Britney" (stv 9.9999997e-06 0.99900001))
       )
    )
)
    (EvaluationLink (mtv (stv 0 0)(vh "CONTEXTUAL" 96).(stv 1 0.3996)) (PredicateNode "friendOf" (stv 1 0))
       (ListLink (ConceptNode "Britney" (stv 9.9999997e-06 0.99900001))
          (ConceptNode "Amir" (stv 9.9999997e-06 0.99900001)))))

