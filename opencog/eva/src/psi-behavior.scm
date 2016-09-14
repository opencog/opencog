;
; psi-behvaior.scm
(use-modules (ice-9 format))


(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog openpsi))
(load "face-priority.scm")

;; ------------------------------------------------------------------
; Demand associated with faces
(define face-demand (psi-demand "face interaction" 1))
(define face-demand-satisfied (True))

; Demand associated with speech interaction
(define speech-demand (psi-demand "speech interaction" 1))
(define speech-demand-satisfied (True))

; Demand for tracking faces
; TODO: make generic for orchestration.
(define track-demand (psi-demand "track demand" 1))
(define track-demand-satisfied (True))

(DefineLink
	(DefinedPredicate "Nothing happening?")
	(NotLink
		(SequentialOr
			(DefinedPredicate "Someone requests interaction?")
			(DefinedPredicate "Did someone arrive?")
			(DefinedPredicate "Did someone leave?")
			(DefinedPredicate "Someone visible?"))))


(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone requests interaction?")))
	(DefinedPredicate "Interaction requested action")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone arrive?")))
	(DefinedPredicate "New arrival sequence")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone leave?")))
	(DefinedPredicate "Someone left action")
	face-demand-satisfied (stv 1 1) face-demand)

; This rule is the old multiple-face tracking rule
; TODO Remove after thoroughly testing behavior on robot.
;(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
;		(DefinedPredicate "Someone visible?")))
;	(DefinedPredicate "Interact with people")
;	face-demand-satisfied (stv 1 1) face-demand)

; TODO: How should rules that could run concurrently be represented, when
; we have action compostion(aka Planning/Orchestration)?
(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone visible?")))
	(DefinedPredicate "Interact with face")
	track-demand-satisfied (stv .5 .5) track-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		; TODO: test the behabior when talking.
		; (Not (DefinedPredicate "chatbot is talking?"))
		(DefinedPredicate "Someone visible?")
		(DefinedPredicate "Time to change interaction")))
	(DefinedPredicate "Change interaction target by priority")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Nothing happening?")))
	(DefinedPredicate "Nothing is happening")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(Not (DefinedPredicate "chatbot started talking?"))
		(Not (DefinedPredicate "Is interacting with someone?"))
		(DefinedPredicateNode "Did someone recognizable arrive?")))
	(DefinedPredicate "Interacting Sequence for recognized person")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot started talking?")))
	(DefinedPredicate "Speech started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot is talking?")))
	(DefinedPredicate "Speech ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot stopped talking?")))
	(DefinedPredicate "Speech ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot started listening?")))
	(DefinedPredicate "Listening started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot is listening?")))
	(DefinedPredicate "Listening ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (SequentialAnd (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot stopped listening?")))
	(DefinedPredicate "Listening ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "Skip Interaction?"))
	(DefinedPredicate "Keep alive")
	speech-demand-satisfied (stv 1 1) speech-demand)
