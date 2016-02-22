;; Like moses-pln-synergy-pm.scm but this version relies on the
;; forward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync #t)
;; (cog-logger-set-stdout #t)
(cog-logger-set-level "debug")

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

(pln-fc if-X-takes-Y-and-Y-contains-Z-then-X-takes-Z)
