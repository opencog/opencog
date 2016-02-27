;; Like moses-pln-synergy-pm.scm but this version relies on the
;; forward chainer.

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

;; Set logger to DEBUG
(use-modules (opencog logger))
(cog-logger-set-level "debug")

(pln-fc if-X-takes-Y-and-Y-contains-Z-then-X-takes-Z)
