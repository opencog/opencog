;; Like moses-pln-synergy-fc.scm but this version relies on the
;; backward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
(cog-logger-set-level! "debug")

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-bc-config.scm")

;; ;; First target
;; (define first-target
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
;; (pln-bc first-target)

;; Second target
(define second-target
   (ImplicationLink
      (LambdaLink
         (TypedVariableLink
            (VariableNode "$X")
            (TypeNode "ConceptNode")
         )
         (AndLink
            (EvaluationLink
               (PredicateNode "take")
               (ListLink
                  (VariableNode "$X")
                  (ConceptNode "treatment-1")
               )
            )
            (EvaluationLink (stv 1 1)
               (PredicateNode "contain")
               (ListLink
                  (ConceptNode "treatment-1")
                  (ConceptNode "compound-A")
               )
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
(pln-bc second-target)

;; Final target
;; (pln-bc moses-model)
