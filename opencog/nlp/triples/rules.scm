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
; -----------------------------------------------------------------
; get-anon-var-id! given a string, return a new unique string
; Given an input string, this returns a new, more-or-less unique
; new string.  Only minimal measures are taken to ensure uniqueness;
; so the result is not "strong". This is intended for use in generating
; "unique" variable names in ImplicatinLinks

(define *anon-var-id* 0)
(define *anon-prefix* "$anon-")
(define (get-anon-var-id! str)
	(set! *anon-var-id* (1+ *anon-var-id*))
	(string-concatenate/shared
		(list *anon-prefix* str "-"
			(call-with-output-string 
				(lambda (port)
					(display *anon-var-id* port)
				)
			)
		)
	)
)

(define (set-anon-prefix! str) (set! *anon-prefix* str))

; -----------------------------------------------------------------

(define (r-ifthen P Q)
	(ImplicationLink  P Q)
)

(define (r-rlx rel a b)
	(let* (
			; av, bv are true if a,b start with $
			(av (eq? #\? (string-ref a 0)))
			(bv (eq? #\? (string-ref b 0)))
			; avn is the variable name to use
			(avn (if av a (get-anon-var-id! a)))
			(bvn (if bv b (get-anon-var-id! b)))
		)
		(define pred
			(EvaluationLink (stv 1 1)
				(DefinedLinguisticRelationshipNode rel)
				(ListLink
					(VariableNode avn)
					(VariableNode bvn)
				)
			)
		)
		(define (lem var wrd)
			(LemmaLink (stv 1 1)
				(VariableNode var)
				(WordNode wrd)
			)
		)
		(cond
			((and av bv) pred)
			((and (not av) bv)
				(list pred (lem avn a))
			)
			((and av (not bv))
				(list pred (lem bvn b))
			)
			((and (not av) (not bv))
				(list pred (lem avn a) (lem bvn b))
			)
		)
	)
)


(r-rlx "_subj" "be" "$var0")   ;; _subj(be, $var0)


