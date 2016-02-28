;
; Personality/Behavior scheme module for the Eva robot.
; Defines a "personality" in terms of a set of configurable behavior
; trees.

(define-module (opencog eva-behavior))

(use-modules (opencog) (opencog atom-types))
(use-modules (opencog eva-model))

; Load various parts....
(load "eva-behavior/behavior.scm")
