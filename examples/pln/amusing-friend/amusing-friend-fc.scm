;; Like moses-pln-synergy-pm.scm but this version relies on the
;; forward chainer.

;; Set logger to DEBUG
(use-modules (opencog logger))
;; (cog-logger-set-sync! #t)
;; (cog-logger-set-stdout! #t)
(cog-logger-set-level! "debug")

;; Load the background knowledge
(load "kb.scm")

;; Load the PLN configuration for this demo
(load "pln-config.scm")

(pln-fc (SetLink people-telling-the-truth-are-honest
                 friends-tend-to-be-honest
                 human-acquainted-tend-to-become-friends
                 people-telling-jokes-are-funny
                 funny-is-loosely-equivalent-to-amusing))
