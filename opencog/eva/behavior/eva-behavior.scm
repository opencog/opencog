;
; Personality/Behavior scheme module for the Eva robot.
; Defines a "personality" in terms of a set of configurable behavior
; trees.

(define-module (opencog eva-behavior))

(use-modules (opencog) (opencog atom-types))
(use-modules (opencog eva-model))

; Load various parts....
; Note: we load cfg-eva.scm by default The other cfg-files can
; over-ride the defaults set there, easily enough. But at least
; cfg-eva gives us enough to work on.
(load "eva-behavior/cfg-tools.scm")
(load "eva-behavior/cfg-eva.scm")
(load "eva-behavior/express.scm")
(load "eva-behavior/primitives.scm")
(load "eva-behavior/movement-api.scm")
(load "eva-behavior/behavior.scm")
(load "eva-behavior/psi-behavior.scm")
(load "eva-behavior/orchestrate.scm")

(define-public (load-eva-config)
"
   load-eva-config

   Load behavior paramters appropriage for the Eva blender model.
"
	(load "eva-behavior/cfg-eva.scm"))

(define-public (load-sophia-config)
"
   load-sophia-config

   Load behavior paramters appropriage for the Sophia blender model.
"
	(load "eva-behavior/cfg-sophia.scm"))
