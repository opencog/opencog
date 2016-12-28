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

(pln-bc moses-model)
