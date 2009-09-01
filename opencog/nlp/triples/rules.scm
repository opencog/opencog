;
; rules.scm
;
; An experimental set of copula/preposition-mangling rules.
;
; Some quick notes on evaluation: conjuncts are evaluated from 
; first to last, and so the order of the terms matters. Terms that
; narrow down the search the most dramatically should come first,
; so as to avoid an overly-broad search of the atomspace.
;
; All of these rules are structured so that a search is performed
; only over sentences That are tagged with a link to the node
; "# APPLY TRIPLE RULES". Since this rule is first, this prevents 
; a search over the entire atomspace.
;
;

(define *anon-var-id* 0)
(define (get-anon-var-id! str)
	(set! *anon-var-id* (1+ *anon-var-id*))
	(string-concatenate/shared
		(list str "-anon-"
			(call-with-output-string 
				(lambda (port)
					(display *anon-var-id* port)
				)
			)
		)
	)
)



(define (r-ifthen P Q)
	(ImplicationLink  P Q)
)

(define (r-rlx rel a b)
	(let* ((av (eq? #\? (string-ref a 0)))
			(bv (eq? #\? (string-ref b 0)))
			(avn 
		)
	)
	(define pred
		(EvaluationLink (stv 1 1)
			(DefinedLinguisticRelationshipNode rel)
			(ListLink
				(VariableNode a)
				(VariableNode b)
			)
		)
	)
	(if (not av)
	)
)


(r-rlx "_subj" "be" "$var0")   ;; _subj(be, $var0)


