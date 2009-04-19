scm
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
; --------------------------------------------------------------
; cog-atom-incr
; Increment count truth value on "atom" by "cnt"
; If the current truth value on the atom is not a CountTruthValue,
; then the truth value is replaced by a CountTruthValue, with the 
; count set to "cnt".
;
; XXX this implementation is slow/wasteful, a native C++ would
; be considerably faster.
;
(define (cog-atom-incr atom cnt) 
	(let* (
			(tv (cog-tv atom))
			(atv (cog-tv->alist tv))
			(mean (assoc-ref atv 'mean))
			(conf (assoc-ref atv 'confidence))
			(count (assoc-ref atv 'count))

			; non-CountTV's will not have a 'count in the a-list
			; so its enough to test for that.
			(ntv (if count
					(cog-new-ctv mean conf (+ count cnt))
					(cog-new-ctv mean conf cnt))
			)
		)
		(cog-set-tv! atom ntv)
	)
)

; --------------------------------------------------------------
; process-rule
; Given an ImplicationLink, apply the implication on the atom space.
; This may generate a list of atoms. Take that list, and manually
; store it in the database.
;
(define (process-rule rule)
	(define triple-list (cog-outgoing-set (cog-ad-hoc "do-implication" rule)))

	; Increment count by 1 on each result.
	(for-each (lambda (atom) (cog-atom-incr atom 1)) triple-list)
	(display "Done running implication\n")

	; Store each resultant atom.
	(for-each (lambda (atom) (cog-ad-hoc "store-atom" atom)) triple-list)
	(display "Done running store\n")
)

; --------------------------------------------------------------
;
; Apply the above proceedure to each ImplicationLink that we have.
(define (do-triples) 

	; First, create all of the verb-preposition pairs that
	; we'll need.
	(for-each 
		(lambda (rule) 
			(cog-outgoing-set (cog-ad-hoc "do-implication" rule))
		)
		prep-rule-list
	)
	(for-each process-rule frame-rule-list)
)

.
exit
