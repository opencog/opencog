;; Like moses-pln-synergy-pm.scm but this version relies on the
;; backward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
(cog-logger-set-level! "debug")

;; Load the background knowledge
(load "kb.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

(define target
(And
   (Evaluation
      (Predicate "will-be-friends")
      (List
         (Concept "Self")
         (Variable "$friend")))
   (Evaluation
      (Predicate "is-amusing")
      (Variable "$friend"))
   (Evaluation
      (Predicate "is-honest")
      (Variable "$friend"))))

(pln-bc target)
