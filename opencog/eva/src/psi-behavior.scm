;
; psi-behvaior.scm
(use-modules (ice-9 format))


(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog openpsi))
;
;; ------------------------------------------------------------------
;; Main loop.
;			(SequentialOr
;			    (DefinedPredicate "Skip Interaction?")
;				(DefinedPredicate "Interaction requested")
;				(DefinedPredicate "New arrival sequence")
;				(DefinedPredicate "Someone left")
;				(DefinedPredicate "Interact with people")
;				(DefinedPredicate "Nothing is happening")
;				(True))

;			;; XXX FIXME chatbot is disengaged from everything else.
;			;; The room can be empty, the head is bored or even asleep,
;			;; but the chatbot is still smiling and yabbering.
;			;; If interaction is turned-off need keep alive gestures
;			(SequentialOr
;				(DefinedPredicate "Speech started?")
;				(DefinedPredicate "Speech ongoing?")
;				(DefinedPredicate "Speech ended?")
;				(DefinedPredicate "Listening started?")
;				(DefinedPredicate "Listening?")
;				(DefinedPredicate "Listening ended?")
;				(SequentialAnd
;				    (DefinedPredicate "Skip Interaction?")
;				    (DefinedPredicate "Keep alive")
;				)
;				(True)
;			)

(define (foobar x)
(display "duuuuuuuude wtf foobar\n")
(display x) (newline)
	(cog-evaluate! x)
 (Node "bad value"))
;;
(define (pred-2-schema pnode-str)
	(DefineLink
		(DefinedSchemaNode pnode-str)
		(ExecutionOutputLink (GroundedSchemaNode "scm: foobar")
			(ListLink (DefinedPredicateNode pnode-str))
)))
;;
(pred-2-schema "Interaction requested action")
(pred-2-schema "New arrival sequence action")
(pred-2-schema "Someone left action")
(pred-2-schema "Interact with people action")
(pred-2-schema "Nothing is happening action")
(pred-2-schema "Speech started? action")
(pred-2-schema "Speech ongoing? action")
(pred-2-schema "Speech ended? action")
(pred-2-schema "Listening started? action")
(pred-2-schema "Listening? action")
(pred-2-schema "Listening ended? action")
(pred-2-schema "Keep alive")
;;
;(DefineLink (DefinedPredicateNode "do-noop") (True))
;(pred-2-schema "do-noop")

(define demand-satisfied (True))
(define speech-demand-satisfied (True))
(define face-demand (psi-demand "face interaction" 1))
(define speech-demand (psi-demand "speech interaction" 1))
(define run-demand (psi-demand "run demand" 1))

;(psi-rule (list (DefinedPredicate "Interaction requested"))(DefinedSchemaNode "Interaction requested action") demand-satisfied (stv 1 1) face-demand)
;(psi-rule (list (DefinedPredicate "New arrival sequence")) (DefinedSchemaNode "New arrival sequence action") demand-satisfied (stv 1 1) face-demand)
;(psi-rule (list (DefinedPredicate "Someone left")) (DefinedSchemaNode "Someone left action") demand-satisfied (stv 1 1) face-demand)
;(psi-rule (list (DefinedPredicate "Interact with people")) (DefinedSchemaNode "Interact with people action") demand-satisfied (stv 1 1) face-demand)
;(psi-rule (list (DefinedPredicate "Nothing is happening")) (DefinedSchemaNode "Nothing is happening action") demand-satisfied (stv 1 1) face-demand)
;(psi-rule (list (DefinedPredicate "Speech started?")) (DefinedSchemaNode "Speech started? action") speech-demand-satisfied (stv 1 1) speech-demand)
;(psi-rule (list (DefinedPredicate "Speech ongoing?")) (DefinedSchemaNode "Speech ongoing? action") speech-demand-satisfied (stv 1 1) speech-demand)
;(psi-rule (list (DefinedPredicate "Speech ended?")) (DefinedSchemaNode "Speech ended? action") speech-demand-satisfied (stv 1 1) speech-demand)
;(psi-rule (list (DefinedPredicate "Skip Interaction?")) (DefinedSchemaNode "Keep alive") speech-demand-satisfied (stv 1 1) speech-demand)


(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone requests interaction?"))
	(DefinedSchemaNode "Interaction requested action")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone arrive?"))
	(DefinedSchemaNode "New arrival sequence")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone left"))
	(DefinedSchemaNode "Someone left action")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Interact with people"))
	(DefinedSchemaNode "Interact with people action")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Nothing is happening"))
	(DefinedSchemaNode "Nothing is happening action")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Speech started?"))
	(DefinedSchemaNode "Speech started? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Speech ongoing?"))
	(DefinedSchemaNode "Speech ongoing? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Speech ended?")) 	
	(DefinedSchemaNode "Speech ended? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Listening started?"))
	(DefinedSchemaNode "Listening started? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Listening?"))
	(DefinedSchemaNode "Listening? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink(DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Listening ended?"))
	(DefinedSchemaNode "Listening ended? action")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "Skip Interaction?"))
	(DefinedSchemaNode "Keep alive")
	speech-demand-satisfied (stv 1 1) speech-demand)

; ----------------------------------------------------------------------
