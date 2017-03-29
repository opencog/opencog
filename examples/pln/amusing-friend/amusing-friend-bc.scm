;; Like moses-pln-synergy-pm.scm but relies on the backward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
(cog-logger-set-level! "debug")

;; Load the background knowledge
(load "kb.scm")

;; Load the PLN configuration for this demo
(load "pln-bc-config.scm")

;; Run the BC on each steps as described in amusing-friend-pm.scm, for
;; debugging.

;; (define step-1
;; (Evaluation
;;   (Predicate "is-honest")
;;   (Concept "Bob"))
;; )
;; (pln-bc step-1)

;; (define step-2
;; (Implication
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y"))))
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$X"))
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$Y")))))
;; )
;; (pln-bc step-2)

;; (define step-3
;; (Lambda
;;   (VariableList
;;     (TypedVariable
;;       (Variable "$X")
;;       (Type "ConceptNode"))
;;     (TypedVariable
;;       (Variable "$Y")
;;      (Type "ConceptNode")))
;;   (Evaluation
;;     (Predicate "will-be-friends")
;;     (List
;;       (Variable "$X")
;;       (Variable "$Y"))))
;; )
;; (pln-bc step-3)

;; ;; Skipped for now
;; (define step-4
;; (Lambda
;;    (VariableList
;;       (TypedVariable
;;          (Variable "$X")
;;          (Type "ConceptNode"))
;;       (TypedVariable
;;          (Variable "$Y")
;;          (Type "ConceptNode")))
;;    (And
;;       (Evaluation
;;          (Predicate "is-honest")
;;          (Variable "$X"))
;;       (Evaluation
;;          (Predicate "is-honest")
;;          (Variable "$Y"))))
;; )
;; (pln-bc step-4)

;; (define step-5
;; (Implication
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$X"))
;;          (Evaluation
;;             (Predicate "is-honest")
;;             (Variable "$Y"))))
;;    (Lambda
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y")))))
;; )
;; (pln-bc step-5)

;; (define step-6
;; (Implication
;;    (LambdaLink
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (And
;;          (Inheritance
;;             (Variable "$X")
;;             (Concept "human"))
;;          (Inheritance
;;             (Variable "$Y")
;;             (Concept "human"))
;;          (Evaluation
;;             (Predicate "acquainted")
;;             (List
;;                (Variable "$X")
;;                (Variable "$Y")))))
;;    (LambdaLink
;;       (VariableList
;;          (TypedVariable
;;             (Variable "$X")
;;             (Type "ConceptNode"))
;;          (TypedVariable
;;             (Variable "$Y")
;;             (Type "ConceptNode")))
;;       (Evaluation
;;          (Predicate "will-be-friends")
;;          (List
;;             (Variable "$X")
;;             (Variable "$Y")))))
;; )
;; (pln-bc step-6)

;; (define step-7
;; (ImplicationLink
;;    (AndLink
;;       (LambdaLink
;;          (VariableList
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (TypedVariableLink
;;                (VariableNode "$Y")
;;                (TypeNode "ConceptNode")
;;             )
;;          )
;;          (AndLink
;;             (EvaluationLink
;;                (PredicateNode "is-honest")
;;                (VariableNode "$X")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "is-honest")
;;                (VariableNode "$Y")
;;             )
;;          )
;;       )
;;       (LambdaLink
;;          (VariableList
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (TypedVariableLink
;;                (VariableNode "$Y")
;;                (TypeNode "ConceptNode")
;;             )
;;          )
;;          (AndLink
;;             (InheritanceLink
;;                (VariableNode "$X")
;;                (ConceptNode "human")
;;             )
;;             (InheritanceLink
;;                (VariableNode "$Y")
;;                (ConceptNode "human")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "acquainted")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (VariableNode "$Y")
;;                )
;;             )
;;          )
;;       )
;;    )
;;    (LambdaLink
;;       (VariableList
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (TypedVariableLink
;;             (VariableNode "$Y")
;;             (TypeNode "ConceptNode")
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "will-be-friends")
;;          (ListLink
;;             (VariableNode "$X")
;;             (VariableNode "$Y")))))
;; )
;; (pln-bc step-7)

(define step-8
(ImplicationLink
   (LambdaLink
      (VariableList
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (TypedVariableLink
            (VariableNode "$Y")
            (TypeNode "ConceptNode")
         )
      )
      (AndLink
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$X")
         )
         (EvaluationLink
            (PredicateNode "is-honest")
            (VariableNode "$Y")
         )
         (InheritanceLink
            (VariableNode "$X")
            (ConceptNode "human")
         )
         (InheritanceLink
            (VariableNode "$Y")
            (ConceptNode "human")
         )
         (EvaluationLink
            (PredicateNode "acquainted")
            (ListLink
               (VariableNode "$X")
               (VariableNode "$Y")
            )
         )
      )
   )
   (AndLink
      (LambdaLink
         (VariableList
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (TypedVariableLink
               (VariableNode "$Y")
               (TypeNode "ConceptNode")
            )
         )
         (AndLink
            (EvaluationLink
               (PredicateNode "is-honest")
               (VariableNode "$X")
            )
            (EvaluationLink
               (PredicateNode "is-honest")
               (VariableNode "$Y")
            )
         )
      )
      (LambdaLink
         (VariableList
            (TypedVariableLink
               (VariableNode "$X")
               (TypeNode "ConceptNode")
            )
            (TypedVariableLink
               (VariableNode "$Y")
               (TypeNode "ConceptNode")
            )
         )
         (AndLink
            (InheritanceLink
               (VariableNode "$X")
               (ConceptNode "human")
            )
            (InheritanceLink
               (VariableNode "$Y")
               (ConceptNode "human")
            )
            (EvaluationLink
               (PredicateNode "acquainted")
               (ListLink
                  (VariableNode "$X")
                  (VariableNode "$Y")))))))
)
(pln-bc step-8)

;; (define target
;; (And
;;    (Evaluation
;;       (Predicate "will-be-friends")
;;       (List
;;          (Concept "Self")
;;          (Variable "$friend")))
;;    (Evaluation
;;       (Predicate "is-amusing")
;;       (Variable "$friend"))
;;    (Evaluation
;;       (Predicate "is-honest")
;;       (Variable "$friend")))
;; )
;; (pln-bc target)
