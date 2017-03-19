;
; movement-api.scm
;
; Definitions providing the movement API.
;
; The definitions here give the API only, but not an implememtation.
; The only current implementation is in the module `(opencog movement)`
; which connects these to ROS blender API robot animations.

; Printer stub
(define-public (prt-pred-defn PRED) 
   (format #t "Called (DefinedPredicate ~a)\n" (cog-name PRED))
   (stv 1 1))

; Delete a definition
(define (delete-definition STR)
	(cog-delete (car
		(cog-get-link 'DefineLink 'DefinedPredicateNode
			(DefinedPredicate STR)))))


(define (mkpred STR)
	(DefineLink
		(DefinedPredicate STR)
		(EvaluationLink
			(GroundedPredicate "scm: prt-pred-defn")
			(ListLink (DefinedPredicate STR)))))
