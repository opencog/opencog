; loading additional dependency
(load-scm-from-file "../opencog/nlp/microplanning/sentence-forms.scm")
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")
(load-scm-from-file "../opencog/nlp/microplanning/anaphora.scm")


; =======================================================================
; Main microplanning functions
; =======================================================================

; -----------------------------------------------------------------------
; microplanning -- The main microplanning function call
;
; Accepts a SequentialAndLink containing a list of atoms to be spoken.
; "utterance-type" is either 'declarative', 'interrogative', 'imperative'
; or 'interjective'
;
(define (microplanning seq-link utterance-type)
	(define chunks '())

	; find the preferred sentence forms base on utterance-type
	; XXX should utterance-type be part of the link instead of passing as parameter
	(define (get-favored-forms)
		; initialize the sentence forms as needed
		(microplanning-init)
		(cog-outgoing-set (car
			(cond ((string=? "declarative" utterance-type)
				(cog-chase-link 'InheritanceLink 'OrLink (ConceptNode "declarative"))
			      )
			      ((string=? "interrogative" utterance-type)
				'()
			      )
			      ((string=? "imperative" utterance-type)
				'()
			      )
			      ((string=? "interjective" utterance-type)
				'()
			      )
			)
		))
	)
	
	(define (call-sureal atoms)
		; wrap a SetLink around the atoms
		(sureal (SetLink atoms))
	)
	(define favored-forms (get-favored-forms))

	(set! chunks (make-sentence-chunks (cog-outgoing-set seq-link) favored-forms))
	
	(cond ((not (null? chunks))
		; insert anaphora
		(set! chunks (insert-anaphora chunks favored-forms))
	
		chunks	
		; call SuReal on each item in sentence-chunks
		;(map call-sureal chunks)
	      )
	)
)

; -----------------------------------------------------------------------
; make-sentence-chunks -- The main helper function for handling a microplanning request
;
; Calls make-sentence repeatedly until all informations have been spoken, or
; no longer say-able.
;
(define (make-sentence-chunks atoms-set favored-forms)
	(define atoms-unused atoms-set)
	(define sentence-chunks '())
	(define (recursive-helper)
		(define new-chunk '())

		; keep calling make sentence until all atoms are used
		(cond ((not (null? atoms-unused))
			(set! new-chunk (make-sentence atoms-unused atoms-set favored-forms))
			(set! sentence-chunks (cons new-chunk sentence-chunks))
			; XXX keep some of the atoms (those that do not satisfy sentence forms) for later use?
			(set! atoms-unused (lset-difference equal? atoms-unused new-chunk))
			(recursive-helper)
		      )
		)
	)

	; loop make-sentence on remaining atoms
	(recursive-helper)

	; reverse since we were adding to the front
	(reverse sentence-chunks)
)

; -----------------------------------------------------------------------
; make-sentence -- Create a single sentence
;
; Greedily add new atoms (using some heuristics) to form a new sentence.
;
(define (make-sentence atoms-unused atoms-set favored-forms)
	; initialize weight bases on "time ordering"
	(define time-weights '())
	; initialize weight (0, 1) bases on whether the atom satisfy a sentence form
	(define form-weights '())
	; initialize weight bases on linkage to other atoms
	(define link-weights '())

	; comb-proc should takes in param "time", "form", "link" in this order
	(define (calc-weights choices bases comb-proc)
		(define choices-len (length choices))

		; helper function: get all nodes for a atom, get all nodes in bases, count how many common nodes
		(define (link-count atom)
			(define atom-nodes (cog-get-all-nodes atom))
			; XXX need to optimize performance here?
			(define used-nodes (delete-duplicates (append-map cog-get-all-nodes bases)))
			(length (lset-intersection equal? atom-nodes used-nodes))
		)

		(set! time-weights (list-tabulate choices-len (lambda (i) (- choices-len i))))
		(set! form-weights (map (lambda (atom) (if (match-sentence-forms atom favored-forms) 1 0)) choices))
		(set! link-weights (map link-count choices))
		
		(display "current weights\n")
		(display time-weights)
		(display "\n")
		(display form-weights)
		(display "\n")
		(display link-weights)
		(display "\nweights done\n")

		; create a combined weight using comb-proc
		(map comb-proc time-weights form-weights link-weights)
	)

	(define atoms-used (lset-difference equal? atoms-set atoms-unused))
	(define atoms-failed '())
	(define chunk '())

	; helper function to sort the atoms list base on weights and choose the highest one
	(define (pick-atom a-list weights)
		(define assoc-list (sort (map cons a-list weights) (lambda (x y) (> (cdr x) (cdr y)))))
		(caar assoc-list)
	)

	; helper function for looping
	(define (recursive-helper atoms-to-try)
		(define head (car atoms-to-try))
		(define result (check-chunk atoms-to-try))

		(define (give-up-unadded-part)
			; atoms that did not work
			(define dead-set (lset-difference atoms-to-try chunk))
			; atoms that worked
			(define good-set (lset-intersection atoms-to-try chunk))

			(set! atoms-failed (append dead-set atoms-failed))
			(set! atoms-unused (lset-difference dead-set atoms-unused))

			; try "saying" the previous working iteration again and re-choose a new atom
			(recursive-helper good-set)
		)
		(define (update-chunk)
			; add anything that has not been included in the chunk yet
			(set! chunk (lset-union equal? atoms-to-try chunk))
			(set! atoms-unused (lset-difference equal? atoms-unused atoms-to-try))
			(display "updating chunk\n")
			(display chunk)
			(display "\nupdating chunk ended\n")
		)

		; temporary variables not all conditions needed
		(define temp-var1 '())
		(define temp-var2 '())
		(define temp-differences '())

		(display "trying atoms\n")
		(display atoms-to-try)
		(display "\n....\n")

		(cond ; not say-able
		      ((= result 0)
			(display "not say-able\n")
			; could be the atoms cannot be said unless bringing in additional atoms (such as "and/or/that")
			; try to add more (up to 3 different links)
			(cond ((<= (length atoms-to-try) 3)
				; look to see if the newest link has any ConceptNode that is solo
				(set! temp-var1 (cog-get-all-nodes head))
				(set! temp-var2 (append-map cog-get-all-nodes (cdr atoms-to-try)))
				(set! temp-differences (filter has-word-inst? (lset-difference temp-var1 temp-var2)))

				(cond ((null? temp-differences)
					(give-up-unadded-part)
				      )
				      (else
					; find the first link in atoms-unused that contains one of the solo word
					(set! temp-var1
						(find (lambda (a)
							(find (lambda (w) (cog-has-node? a w)) temp-differences)
						     )
						     atoms-unused
						)
					)

					; if an atom with the solo word exists
					(if (temp-var1)
						(recursive-helper (cons temp-var1 atoms-to-try))
						(give-up-unadded-part)
					)
				      )
				)
			      )
			      ; give up on all the atoms not "say-able"
			      (else
				(give-up-unadded-part)
			      )
			)
		      )
		      ; not long/complex enough
		      ((= result 1)
			(display "result not long enough\n")
			; add the currently "say-able" stuff to our chunk
			(update-chunk)

			; only continue if there are more to say
			(if (not (null? atoms-unused))
				(recursive-helper
					(cons
						; weight is (time-weights + link-weights) * (2 - form-weights), ie. prefer links that do not
						; satisify a sentence form, meaning it cannot be "said" on its own.
						(pick-atom atoms-unused (calc-weights atoms-unused chunk (lambda (t f l) (* (+ t l) (- 2 f)))))
						atoms-to-try
					)
				)
			)
		      )
		      ; too long/complex
		      (else
			(display "result too long\n")
			(update-chunk)
		      )
		)
	)

	; the initial critera for choosing a starting point would be (time-weights + link-weights) * form-weights
	(recursive-helper (list (pick-atom atoms-unused (calc-weights atoms-unused atoms-used (lambda (t f l) (* (+ t l) f))))))

	; return the sentence chunk (reverse because we've been adding to the front)
	(reverse chunk)
)



;;;;;;;; brain organizing stuff 

; if not long enough
; add external links that share a node? how to determine what to include?
; add the next highest weight atom?
;    prefer atoms with low form-weights & high link weights (with the new chunk, not the old stuff said)?


; some atoms used can be leave out of used to be reused later?
; eg. atoms that do not satisfy a sentence form?






;; base on utterance-type, pick a sentence form?
;; do we need some sort of template in the atomspace to tell what SVO looks like?
;; try hard-coding first?

;;; loop
;; assuming SV or SVO, pick an atom with link to a verb node (PartOfSpeechLink)

; look at atoms-unused and find first node that is verb?
; or, find all nodes that is verb, then use the list index as a weight
; then check how many outgoing links connect to a "used" set?

; highest weight, try to say


;; get the root link
;; check if it is "sayable"
;; if sentence not long enough, greedily add another link (closest link in the saying set)
;; check if "sayable" again, or too long, and discard the new addition if necessary
;; update used atoms and loop

;; return a set of atoms used
