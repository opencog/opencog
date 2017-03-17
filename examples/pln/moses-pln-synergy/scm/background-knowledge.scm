;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; General background knowledge ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; If X takes Y and Y contains Z, then X takes Z
(define if-X-takes-Y-and-Y-contains-Z-then-X-takes-Z
   (ImplicationScopeLink (stv 1 1)
      (VariableList
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode"))
         (TypedVariableLink
            (VariableNode "$Y")
            (TypeNode "ConceptNode"))
         (TypedVariableLink
            (VariableNode "$Z")
            (TypeNode "ConceptNode")))
      (AndLink
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (VariableNode "$Y")))
         (EvaluationLink
            (PredicateNode "contain")
            (ListLink
               (VariableNode "$Y")
               (VariableNode "$Z"))))
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (VariableNode "$X")
            (VariableNode "$Z"))))
   )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Background knowledge about John ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; This is essentially dummy, just to make partial implication
;; instantiation works till partial pattern matcher (probably via self
;; grounding is supported).
(EvaluationLink (stv 1 1)
   (PredicateNode "take")
   (ListLink
      (ConceptNode "John")
      (ConceptNode "treatment-1")))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Background knowledge about treatment-1 ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "take-treatment-1" (stv 0.1 0.8))
(PredicateNode "take-compound-A" (stv 0.2 0.8))

;; take-treatment-1(X) is equivalent to take(X, treatment-1)
(define take-treatment-1-X-is-equivalent-to-take-X-treatment-1
   (EquivalenceLink (stv 1 1)
      (PredicateNode "take-treatment-1")
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode"))
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
   )
)
;; take-compound-A(X) is equivalent to take(X, compound-A)
(define take-compound-A-X-is-equivalent-to-take-X-compound-A
   (EquivalenceLink (stv 1 1)
      (PredicateNode "take-compound-A")
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode"))
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "compound-A")
            )
         )
      )
   )
)

;; treatment-1 contains compound-A
(EvaluationLink (stv 1 1)
   (PredicateNode "contain")
   (ListLink
      (ConceptNode "treatment-1")
      (ConceptNode "compound-A")
   )
)

;; Taking compound-A tends middly to speed up recovery of injury
;; alpha, but we're highly confident about it.
(ImplicationLink (stv 0.55 0.9)
   (PredicateNode "take-compound-A")
   (PredicateNode "recovery-speed-of-injury-alpha")
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Background knowledge about treatment-2 ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "take-treatment-2" (stv 0.05 0.8))

;; take-treatment-2(X) is equivalent to take(X, treatment-2)
(EquivalenceLink (stv 1 1)
   (PredicateNode "take-treatment-2")
   (LambdaLink
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode"))
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (VariableNode "$X")
            (ConceptNode "treatment-2")
         )
      )
   )
)

;; take-compound-B(X) is equivalent to take(X, compound-B)
(EquivalenceLink (stv 1 1)
   (PredicateNode "take-compound-B")
   (LambdaLink
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode"))
      (EvaluationLink
         (PredicateNode "take")
         (ListLink
            (VariableNode "$X")
            (ConceptNode "compound-B")
         )
      )
   )
)

;; treatment-2 contains compound-B
(EvaluationLink (stv 0.99 0.99)
   (PredicateNode "contain")
   (ListLink
      (ConceptNode "treatment-2")
      (ConceptNode "compound-B")
   )
)

;; Taking compound-B tends to greatly speed up recovery of injury alpha
;; (but we're not very confident about it)
(ImplicationLink (stv 0.8 0.6)
   (PredicateNode "take-compound-B")
   (PredicateNode "recovery-speed-of-injury-alpha")
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Background knowledge about eating fruits and vegetables ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "eat-lots-fruits-vegetables" (stv 0.07 0.8))
(PredicateNode "is-well-hydrated")

;; Individuals that eat a lot of fruits and vegetables are well hydrated
(ImplicationLink (stv 0.85 0.95)
   (PredicateNode "eat-lots-fruits-vegetables")
   (PredicateNode "is-well-hydrated")
)

;; Being well hydrated tends to speed up injury recovery
(define being-well-hydrated-tends-to-speed-up-injury-recovery
   (ImplicationScopeLink (stv 0.7 0.97)
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "PredicateNode"))
      (MemberLink
         (VariableNode "$X")
         (ConceptNode "injury-recovery-speed-predicates"))
      (ImplicationLink
         (PredicateNode "is-well-hydrated")
         (VariableNode "$X"))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Background knowledge about the target feature ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(PredicateNode "recovery-speed-of-injury-alpha" (stv 0.3 0.8))

;; recovery-speed-injury-alpha is in the class
;; injury-recovery-of-speed-predicates
(MemberLink (stv 1 1)
   (PredicateNode "recovery-speed-of-injury-alpha")
   (ConceptNode "injury-recovery-speed-predicates")
)
