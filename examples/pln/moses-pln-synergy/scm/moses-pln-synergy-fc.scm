;; Like moses-pln-synergy-pm.scm but this version relies on the
;; forward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
;; (cog-logger-set-level! "debug")

;; Load MOSES model
(load "moses-model.scm")

;; Load the background knowledge
(load "background-knowledge.scm")

;; Load the PLN configuration for this demo
(load "pln-fc-config.scm")

(pln-fc (SetLink if-X-takes-Y-and-Y-contains-Z-then-X-takes-Z
                 take-treatment-1-X-is-equivalent-to-take-X-treatment-1
                 take-compound-A-X-is-equivalent-to-take-X-compound-A
                 being-well-hydrated-tends-to-speed-up-injury-recovery))
