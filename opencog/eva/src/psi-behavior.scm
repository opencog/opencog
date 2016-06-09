;
; psi-behvaior.scm
(use-modules (ice-9 format))


(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog openpsi))
;
;; ------------------------------------------------------------------

; XXX FIXME -- terrible errible hack -- mostly because OpenPsi
; is expecting actions to be schema, and not predicates.
(define (foobar x)
; (display "duuuuuuuude foobar pred-schema wrapper\n")
; (display x) (newline)
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
(pred-2-schema "New arrival sequence")
(pred-2-schema "Someone left action")
(pred-2-schema "Interact with people")
(pred-2-schema "Nothing is happening")
(pred-2-schema "Speech started")
(pred-2-schema "Speech ongoing")
(pred-2-schema "Speech ended")
(pred-2-schema "Listening started")
(pred-2-schema "Listening ongoing")
(pred-2-schema "Listening ended")
(pred-2-schema "Keep alive")
;;
;(DefineLink (DefinedPredicateNode "do-noop") (True))
;(pred-2-schema "do-noop")

(define face-demand-satisfied (True))
(define speech-demand-satisfied (True))
(define face-demand (psi-demand "face interaction" 1))
(define speech-demand (psi-demand "speech interaction" 1))
; (define run-demand (psi-demand "run demand" 1))

(DefineLink
	(DefinedPredicate "Nothing happening?")
	(NotLink
		(SequentialOr
			(DefinedPredicate "Someone requests interaction?")
			(DefinedPredicate "Did someone arrive?")
			(DefinedPredicate "Did someone leave?")
			(DefinedPredicate "Someone visible?"))))


(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone requests interaction?"))
	(DefinedSchemaNode "Interaction requested action")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone arrive?"))
	(DefinedSchemaNode "New arrival sequence")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone leave?"))
	(DefinedSchemaNode "Someone left action")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone visible?"))
	(DefinedSchemaNode "Interact with people")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Nothing happening?"))
	(DefinedSchemaNode "Nothing is happening")
	face-demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot started talking?"))
	(DefinedSchemaNode "Speech started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot is talking?"))
	(DefinedSchemaNode "Speech ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot stopped talking?"))
	(DefinedSchemaNode "Speech ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot started listening?"))
	(DefinedSchemaNode "Listening started")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot is listening?"))
	(DefinedSchemaNode "Listening ongoing")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "chatbot stopped listening?"))
	(DefinedSchemaNode "Listening ended")
	speech-demand-satisfied (stv 1 1) speech-demand)

(psi-rule (list (DefinedPredicate "Skip Interaction?"))
	(DefinedSchemaNode "Keep alive")
	speech-demand-satisfied (stv 1 1) speech-demand)

; ----------------------------------------------------------------------

; There MUST be a DefinedPredicateNode with exactly the name
; below in order for psi-run to work. Or we could just blow
; that off, and use our own loop...
(define loop-name (string-append psi-prefix-str "loop"))
(DefineLink
	(DefinedPredicate loop-name)
	(SatisfactionLink
		(SequentialAnd
			(Evaluation (GroundedPredicate "scm: psi-step")
				(ListLink))
			(Evaluation (GroundedPredicate "scm: psi-run-continue?")
				(ListLink))
			; If ROS is dead, or the continue flag not set,
			; then stop running the behavior loop.
			(DefinedPredicate "ROS is running?")
			(DefinedPredicate loop-name))))

; ----------------------------------------------------------------------
