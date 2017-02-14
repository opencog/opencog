;; Like moses-pln-synergy-fc.scm but this version relies on the
;; backward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
(cog-logger-set-timestamp! #f)
(cog-logger-set-level! "debug")

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-bc-config.scm")

;; Target (1)
;; (define target-1
;;    (ImplicationScopeLink
;;       (TypedVariableLink
;;          (VariableNode "$X")
;;          (TypeNode "ConceptNode")
;;       )
;;       (AndLink
;;          (EvaluationLink
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "treatment-1")
;;             )
;;          )
;;          (EvaluationLink (stv 1 0.99999982)
;;             (PredicateNode "contain")
;;             (ListLink
;;                (ConceptNode "treatment-1")
;;                (ConceptNode "compound-A")
;;             )
;;          )
;;       )
;;       (EvaluationLink
;;          (PredicateNode "take")
;;          (ListLink
;;             (VariableNode "$X")
;;             (ConceptNode "compound-A")
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-1)

;; Target (2)
;; (define target-2
;;    (ImplicationLink
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (AndLink
;;             (EvaluationLink
;;                (PredicateNode "take")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (ConceptNode "treatment-1")
;;                )
;;             )
;;             (EvaluationLink (stv 1 1)
;;                (PredicateNode "contain")
;;                (ListLink
;;                   (ConceptNode "treatment-1")
;;                   (ConceptNode "compound-A")
;;                )
;;             )
;;          )
;;       )
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (EvaluationLink
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "compound-A")
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-2)

;; Target (3) is not useful for the BC

;; Target (4)
;; (define target-4
;;    (LambdaLink
;;       (TypedVariableLink
;;          (VariableNode "$X")
;;          (TypeNode "ConceptNode")
;;       )
;;       (EvaluationLink
;;          (PredicateNode "contain")
;;          (ListLink
;;             (ConceptNode "treatment-1")
;;             (ConceptNode "compound-A")
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-4)

;; Target (5)
;; (define target-5
;;    (ImplicationLink
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (EvaluationLink
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "treatment-1")
;;             )
;;          )
;;       )
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (EvaluationLink
;;             (PredicateNode "contain")
;;             (ListLink
;;                (ConceptNode "treatment-1")
;;                (ConceptNode "compound-A")
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-5)

;; Target (6)
;; (define target-6
;;    (ImplicationLink
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (EvaluationLink
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "treatment-1")
;;             )
;;          )
;;       )
;;       (AndLink
;;          (LambdaLink
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "take")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (ConceptNode "treatment-1")
;;                )
;;             )
;;          )
;;          (LambdaLink
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "contain")
;;                (ListLink
;;                   (ConceptNode "treatment-1")
;;                   (ConceptNode "compound-A")
;;                )
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-6)

;; Target (7)
;; (define target-7
;;    (ImplicationLink
;;       (AndLink
;;          (LambdaLink
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "take")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (ConceptNode "treatment-1")
;;                )
;;             )
;;          )
;;          (LambdaLink
;;             (TypedVariableLink
;;                (VariableNode "$X")
;;                (TypeNode "ConceptNode")
;;             )
;;             (EvaluationLink
;;                (PredicateNode "contain")
;;                (ListLink
;;                   (ConceptNode "treatment-1")
;;                   (ConceptNode "compound-A")
;;                )
;;             )
;;          )
;;       )
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (AndLink
;;             (EvaluationLink
;;                (PredicateNode "take")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (ConceptNode "treatment-1")
;;                )
;;             )
;;             (EvaluationLink
;;                (PredicateNode "contain")
;;                (ListLink
;;                   (ConceptNode "treatment-1")
;;                   (ConceptNode "compound-A")
;;                )
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-7)

;; Target (8)
;; (define target-8
;;    (ImplicationLink
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (EvaluationLink
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "treatment-1")
;;             )
;;          )
;;       )
;;       (LambdaLink
;;          (TypedVariableLink
;;             (VariableNode "$X")
;;             (TypeNode "ConceptNode")
;;          )
;;          (AndLink
;;             (EvaluationLink
;;                (PredicateNode "take")
;;                (ListLink
;;                   (VariableNode "$X")
;;                   (ConceptNode "treatment-1")
;;                )
;;             )
;;             (EvaluationLink
;;                (PredicateNode "contain")
;;                (ListLink
;;                   (ConceptNode "treatment-1")
;;                   (ConceptNode "compound-A")
;;                )
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-8)

;; Target (9)
(define target-9
   (ImplicationLink
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (EvaluationLink
            (PredicateNode "take")
            (ListLink
               (VariableNode "$X")
               (ConceptNode "treatment-1")
            )
         )
      )
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
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
(pln-bc target-9)

;; Final target
;; (pln-bc moses-model)
