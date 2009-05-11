scm
;
; collect-stats.scm
;
; Process parsed text through the prepositional-triples code.
;
; This will run the preposition-triple rules through the forward
; chainer, and then go through the results, updating the 
; CountTruthValue associated with each, and then storing the 
; updated count in the OpenCog persistence backend.
;
; Linas Vepstas April 2009
;
; (use-modules (ice-9 rdelim))
; (use-modules (ice-9 popen))

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
	; (system "date")
	; (system "echo Done running one implication\n")

	; Store each resultant atom.
	(for-each (lambda (atom) (cog-ad-hoc "store-atom" atom)) triple-list)
)

; --------------------------------------------------------------
;
; Apply the above proceedure to each ImplicationLink that we have.
; 
(define (fire-all-triple-rules) 

	; First, create all of the preposition phrases that
	; we'll need.
	(for-each 
		(lambda (rule) 
			; (system "date")
			; (system "echo Start working on a prep rule\n")
			(cog-outgoing-set (cog-ad-hoc "do-implication" rule))
		)
		prep-rule-list
	)
	; (system "date")
	; (system "echo Start working on a triple rules\n")
	(for-each process-rule frame-rule-list)
)

.
exit
