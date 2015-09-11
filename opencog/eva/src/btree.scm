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
#|
(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(PresentLink (ListLink room-state (VariableNode "$x")))
			(EqualLink (VariableNode "$x") room-empty)
			(EvaluationLink
				(GroundedPredicateNode "src: print-msg")
				(ListLink))
		)))

(cog-satisfy empty-seq)
|#

; ------------------------------------------------------
