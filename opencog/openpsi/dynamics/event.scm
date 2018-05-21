;
; event.scm
; Event Detection - ... ??
;
; Copyright (c) 2106 OpenCog Foundation
;
; --------------------------------------------------------------
; Event Detection
;
; Event detection works by monitoring the timestamp of the event's most
; recent occurrence, which is stored via StateLink. Eventually, this
; should be evaluated through use of the time server. An historical
; "previous" most recent event ts is also stored, so we can know when
; a new event has occurred when the "current" most recent ts differs
; from the "previous" most recent ts. In essence, the "value" of the
; event being monitored for change is the ts of its most recent
; occurrence. The current approach relies on something outside of
; OpenPsi to set the most-recent-event-ts each time the event occurs.
; One place where this can happen is through an event-detection callback
; (see psi-set-event-callback), ; which is called at each step of the
; psi-dynamics loop.
;
; When a particular instance of an event is detected, the value of the
; event is set to 1 for a single psi-loop step and only for a single
; psi-loop step.
;
; Instructions
;
; Create monitored events with psi-create-monitored-event function in
; event.scm.
;
; Create event detection callback(s) with  `psi-set-event-callback!`
; in `updater.scm`. The callback function should indicate that a
; particular event has occurred by calling `psi-set-event-occurrence!`,
; which uses a StateLink to represent the most recent occurrence of
; the event.

(define psi-event-node (Concept (string-append psi-prefix-str "event")))

(define (psi-create-monitored-event event)
	(define event-concept (Concept (string-append psi-prefix-str event)))
	(Inheritance event-concept psi-event-node)
	; Initialize value to 0, when an instance of the event occurs, value will be
	; set to 1.
	(psi-set-value! event-concept 0)
	;(format #t "new event: ~a\n" event-concept)
	event-concept)

(define (psi-get-monitored-events)
"
  Returns a list containing all the monitored events.
"
    (filter
        (lambda (x) (not (equal? x psi-event-node)))
        (cog-chase-link 'InheritanceLink 'ConceptNode psi-event-node))
)



(define psi-most-recent-occurrence-pred
    (Predicate "psi-most-recent-occurrence"))

(define (psi-set-event-occurrence! event)
    (State
        (List
            event
            psi-most-recent-occurrence-pred)
        (TimeNode (number->string (current-time)))))
