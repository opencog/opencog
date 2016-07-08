; Todo: Move this to a event.scm file
; --------------------------------------------------------------
; Event Detection
(define psi-event-node (Concept (string-append psi-prefix-str "event")))

(define-public (psi-get-monitored-events)
"
  Returns a list containing all the monitored events.
"
    (filter
        (lambda (x) (not (equal? x psi-event-node)))
        (cog-chase-link 'InheritanceLink 'ConceptNode psi-event-node))
)



(define (psi-create-monitored-event event)
	(define event-concept (Concept (string-append psi-prefix-str event)))
	(Inheritance event-concept psi-event-node)
	; Initialize value to 0, when an instance of the event occurs, value will be
	; set to 1.
	(psi-set-value! event-concept 0)
	;(format #t "new event: ~a\n" event-concept)
	event-concept)

(define (psi-create-event-detected-eval event)
"
  Creates a new evaluation of the form:
    Evaluation (stv 0 1)
        Predicate "event-detected"
        List
            event
  stv strength is set to 0 by default
"
	(psi-new-event-eval-with-tv event (stv 0 1)))


(define (psi-new-event-eval event)
	(Evaluation
        psi-event-detected-pred
        (List event)))

(define (psi-new-event-eval-with-tv event tv)
	(cog-set-tv! (psi-new-event-eval event) tv))


; --------------------------------------------------------------
; Event Creation
(define new-face (psi-create-monitored-event "new-face"))
(define speech-giving-starts
	(psi-create-monitored-event "speech-giving-starts"))
(define positive-sentiment-dialog
	(psi-create-monitored-event "positive-sentiment-dialog"))
(define negative-sentiment-dialog
	(psi-create-monitored-event "negative-sentiment-dialog"))


; Shortcuts for dev purposes
(define s speech-giving-starts)
(define pos positive-sentiment-dialog)
(define neg negative-sentiment-dialog)
(define nf new-face)
