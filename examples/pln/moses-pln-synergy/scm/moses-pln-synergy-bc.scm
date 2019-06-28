;; Like moses-pln-synergy-fc.scm but this version relies on the
;; backward chainer.

(use-modules (opencog ure))

;; Set logger to DEBUG
;; (ure-logger-set-sync! #t)
;; (ure-logger-set-stdout! #t)
(ure-logger-set-timestamp! #f)
(ure-logger-set-level! "debug")

;; Set random seed
(use-modules (opencog randgen))
(cog-randgen-set-seed! 100)

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

;; ;; Target (9)
;; (define target-9
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
;;             (PredicateNode "take")
;;             (ListLink
;;                (VariableNode "$X")
;;                (ConceptNode "compound-A")
;;             )
;;          )
;;       )
;;    )
;; )
;; (pln-bc target-9)

;; ;; Target (10)
;; (define target-10
;;   (ImplicationLink
;;      (PredicateNode "is-well-hydrated")
;;      (PredicateNode "recovery-speed-of-injury-alpha")
;;   )
;; )
;; (pln-bc target-10)

;; ;; Target (11-a)
;; (define target-11-a
;;   (ImplicationLink
;;     (LambdaLink
;;       (TypedVariableLink
;;         (VariableNode "$X")
;;         (TypeNode "ConceptNode")
;;       )
;;       (EvaluationLink
;;         (PredicateNode "take")
;;         (ListLink
;;           (VariableNode "$X")
;;           (ConceptNode "compound-A")
;;         )
;;       )
;;     )
;;     (PredicateNode "take-compound-A")
;;   )
;; )
;; (pln-bc target-11-a)

;; ;; Target (11-b)
;; (define target-11-b
;;   (ImplicationLink
;;     (PredicateNode "take-treatment-1")
;;     (LambdaLink
;;       (TypedVariableLink
;;         (VariableNode "$X")
;;         (TypeNode "ConceptNode")
;;       )
;;       (EvaluationLink
;;         (PredicateNode "take")
;;         (ListLink
;;           (VariableNode "$X")
;;           (ConceptNode "treatment-1")
;;         )
;;       )
;;     )
;;   )
;; )
;; (pln-bc target-11-b)

;; Target (12)
(define target-12
  (ImplicationLink
    (PredicateNode "take-treatment-1")
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
(pln-bc target-12)

;; Final target
;; (pln-bc moses-model)
