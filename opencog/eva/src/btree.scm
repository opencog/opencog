;
; btree.scm
;
; Experimental behavior tree in the atomspace.
;

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))
(use-modules (opencog query))
(use-modules (opencog exec))

(load-from-path "utilities.scm")

; ------------------------------------------------------
; Same as in eva-fsm.scm

; Is the room empty, or is someone in it?
(define room-state (AnchorNode "Room State"))
(define room-empty (ConceptNode "room empty"))
(define room-nonempty (ConceptNode "room nonempty"))

;; Assume room empty at first
(ListLink room-state room-empty)


(define (print-msg) (display "triggered\n") (stv 1 1))
; ------------------------------------------------------
;;
;; Below are several different ways of determining if the
;; atomspace contains the link
;; (ListLink (AnchorNode "Room State") (ConceptNode "room empty"))
;; If the atomspace does contain the above, then the print-msg
;; function is run.
; ------------------------------------------------------

;; This variant uses PresentLink to place the room-state into
;; a variable, and then uses EqualLink to check the value of
;; that variable.  This demo will not work right now, because
;; PresentLink is not implemented (github bug #218)
;;
;; This variant haas an advantage over the next one, as it requires
;; only one invocation of the pattern matcher, not two.
#|
(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(PresentLink (ListLink room-state (VariableNode "$x")))
			(EqualLink (VariableNode "$x") room-empty)
			(EvaluationLink
				(GroundedPredicateNode "scm: print-msg")
				(ListLink))
		)))

(cog-satisfy empty-seq)
|#

; ------------------------------------------------------
;; This variant uses a GetLink to fetch the room-state from the
;; AtomSpace, and then uses EqualLink to see if it is in the desired
;; state. Note that this results in *two* invocations of the pattern
;; matcher; the GetLink being the inner one.  Note also that the
;; GetLink returns it's results in a SetLink, so comparison must
;; use a SetLink as well.

(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(EqualLink
				(SetLink room-empty)
				(GetLink (ListLink room-state (VariableNode "$x"))))
			(EvaluationLink
				(GroundedPredicateNode "scm: print-msg")
				(ListLink))
		)))

(cog-satisfy empty-seq)
; ------------------------------------------------------
