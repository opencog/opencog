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

(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))

;; Assume room empty at first
(ListLink room-state room-empty)
(ListLink soma-state soma-sleeping)


(define (print-msg) (display "Triggered\n") (stv 1 1))
(define (print-atom atom) (format #t "Triggered: ~a \n" atom) (stv 1 1))
; ------------------------------------------------------
;;
;; determining if the
;; atomspace contains the link
;; (ListLink (AnchorNode "Room State") (ConceptNode "room empty"))
;; If the atomspace does contain the above, then the print-msg
;; function is run.
; ------------------------------------------------------
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
|#


(DefineLink
	(DefinedPredicateNode "is-empty")
	(EqualLink
		(SetLink room-empty)
		(GetLink (ListLink room-state (VariableNode "$x")))
	))

(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "is-empty")
			(EvaluationLink
				(GroundedPredicateNode "scm: print-msg")
				(ListLink))
		)))

#|

(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			(PresentLink (ListLink room-state (VariableNode "$x")))
			(EqualLink (VariableNode "$x") room-empty)
			(EvaluationLink
				(GroundedPredicateNode "scm: print-msg")
				(ListLink (VariableNode "$x")))

			(PresentLink (ListLink soma-state (VariableNode "$y")))
			(EqualLink (VariableNode "$y") soma-sleeping)
			(EvaluationLink
				(GroundedPredicateNode "scm: print-msg")
				(ListLink (VariableNode "$y")))
		)))

|#
(cog-satisfy empty-seq)

