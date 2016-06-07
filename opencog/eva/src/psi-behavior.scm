;
; psi-behvaior.scm
(use-modules (ice-9 format))


(add-to-load-path "/usr/local/share/opencog/scm")
(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog openpsi))
;
;; ------------------------------------------------------------------

(define (foobar x)
(display "duuuuuuuude foobar pred-schema wrapper\n")
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

(define demand-satisfied (True))
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
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone arrive?"))
	(DefinedSchemaNode "New arrival sequence")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Did someone leave?"))
	(DefinedSchemaNode "Someone left action")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Someone visible?"))
	(DefinedSchemaNode "Interact with people")
	demand-satisfied (stv 1 1) face-demand)

(psi-rule (list (NotLink (DefinedPredicate "Skip Interaction?"))
		(DefinedPredicate "Nothing happening?"))
	(DefinedSchemaNode "Nothing is happening")
	demand-satisfied (stv 1 1) face-demand)

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
