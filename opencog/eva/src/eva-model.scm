;
; Self-awareness scheme module for the Eva robot.
; Alls the self-awareness code to be imported as a module.

(define-module (opencog eva-model))

(use-modules (opencog) (opencog atom-types)
	(opencog query) (opencog exec))


; Load various parts....
(load "eva-model/self-model.scm")
