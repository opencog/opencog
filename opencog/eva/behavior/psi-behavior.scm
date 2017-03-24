;
; psi-behvaior.scm
(use-modules (ice-9 format))


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

; Demand for saliency tracking
(define salient-demand (psi-demand "salient demand" 1))
(define salient-demand-satisfied (True))

; Demand for contorl with web-ui
(define update-demand (psi-demand "update demand" 1))
(define update-demand-satisfied (True))

(DefineLink
	(DefinedPredicate "Nothing happening?")
	(NotLink
		(SequentialOr
			(DefinedPredicate "Someone requests interaction?")
			(DefinedPredicate "Did someone arrive?")
			(DefinedPredicate "Did someone leave?")
			(DefinedPredicate "Someone visible?"))))

(DefineLink
	(DefinedPredicate "Nothing New?")
	(NotLink
		(SequentialOr
			(DefinedPredicate "Someone requests interaction?")
			(DefinedPredicate "Did someone arrive?")
			(DefinedPredicate "Did someone leave?")
			)))

(psi-rule (list (DefinedPredicate "Did Someone New Speak?"))
	(DefinedPredicate "Request interaction with person who spoke")
	face-demand-satisfied (stv 1 1) face-demand)


(psi-rule (list (DefinedPredicate "Someone requests interaction?"))
	(DefinedPredicate "Interaction requested action")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (DefinedPredicate "Did someone arrive?"))
	(DefinedPredicate "New arrival sequence")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (DefinedPredicate "Did someone leave?"))
	(DefinedPredicate "Someone left action")
	face-demand-satisfied (stv 1 1) face-demand)

; This rule is the old multiple-face tracking rule
; TODO Remove after thoroughly testing behavior on robot.
(psi-rule (list (SequentialAnd
		(NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone visible?")))
	(DefinedPredicate "Interact with people")
	face-demand-satisfied (stv 1 1) face-demand)

; TODO: How should rules that could run concurrently be represented, when
; we have action compostion(aka Planning/Orchestration)?
(psi-rule (list (DefinedPredicate "Someone visible?"))
	(DefinedPredicate "Interact with face")
	track-demand-satisfied (stv .5 .5) track-demand)

(psi-rule (list (SequentialAnd
		; TODO: test the behabior when talking.
		; (Not (DefinedPredicate "chatbot is talking?"))
		(DefinedPredicate "Someone visible?")
		(DefinedPredicate "Time to change interaction")))
	(DefinedPredicate "Change interaction target by priority")
	face-demand-satisfied (stv 1 1) face-demand)

;(psi-rule (list (DefinedPredicate "Nothing happening?"))
;	(DefinedPredicate "Nothing is happening")
;	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (SequentialAnd
		(Not (DefinedPredicate "chatbot started talking?"))
		(Not (DefinedPredicate "Is interacting with someone?"))
		(DefinedPredicateNode "Did someone recognizable arrive?")))
	(DefinedPredicate "Interacting Sequence for recognized person")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (DefinedPredicate "chatbot started talking?"))
	(DefinedPredicate "Speech started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "chatbot is talking?"))
	(DefinedPredicate "Speech ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "chatbot stopped talking?"))
	(DefinedPredicate "Speech ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "chatbot started listening?"))
	(DefinedPredicate "Listening started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "chatbot is listening?"))
	(DefinedPredicate "Listening ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "chatbot stopped listening?"))
	(DefinedPredicate "Listening ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "ROS is running?"))
	(DefinedPredicate "Keep alive")
	speech-demand-satisfied (stv 1 1) speech-demand)

;heard loud voice not working
;(psi-rule (list (DefinedPredicate "Heard Loud Voice?"))
;	(DefinedPredicate "Say whoa!")
;	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "Heard Loud Sound?"))
	(DefinedPredicate "Say whoa!")
	speech-demand-satisfied (stv 1 1) speech-demand)

;; saliency tracking
(psi-set-controlled-rule
	(psi-rule (list (True))
		(DefinedPredicate "Salient:Curious")
		salient-demand-satisfied (stv 1 1) salient-demand "saliency-tracking"))

(psi-rule (list (DefinedPredicate "Room bright?"))
	(DefinedPredicate "Bright:happy")
	face-demand-satisfied (stv 1 1) face-demand)

;below doesn't work
;(psi-rule (list (DefinedPredicate "Heard Something?"))
;	(DefinedPredicate "React to Sound")
;	speech-demand-satisfied (stv 1 1) speech-demand)
