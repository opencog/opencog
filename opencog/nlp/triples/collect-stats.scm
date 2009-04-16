;
; collect-stats.scm
;
; Process parsed text through the prepositional-triples code.
;
; This will run the preposition-triple rules through the forward
; chainer, and then go through the results, updating the 
; CountTruthValue assoiated with each, and then storing the 
; updated count in the OpenCog persistence backend.
;
; Linas Vepstas April 2009
;

; process-rule
; Given an ImplicationLink, apply the implication on the atom space.
; This may generate a list of atoms. Take that list, and manually
; store it in the database.
;
(define (process-rule rule)
	(define triple-list (cog-outgoing-set (cog-ad-hoc "do-implication" rule)))
	(define (store-stats atom) (cog-ad-hoc "store-atom" atom))

	; Store each resultant atom.
	(for-each store-stats triple-list)
)

; Apply the above proceedure to each ImplicationLink that we have.
(for-each process-rule frame-rule-list)
