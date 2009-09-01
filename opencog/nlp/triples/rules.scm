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

(define (r-ifthen P Q)
	(ImplicationLink  P Q)
)


