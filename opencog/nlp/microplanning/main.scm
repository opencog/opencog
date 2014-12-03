; loading additional dependency
(load-scm-from-file "../opencog/nlp/microplanning/sentence-forms.scm")
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")
(load-scm-from-file "../opencog/nlp/microplanning/anaphora.scm")
(load-scm-from-file "../opencog/nlp/microplanning/chunks-option.scm")
(load-scm-from-file "../opencog/nlp/microplanning/chunks-set.scm")

; =======================================================================
; Some contants
; =======================================================================

(define *microplanning_not_sayable* 0)
(define *microplanning_sayable* 1)
(define *microplanning_too_long* 2)
(define *default_chunks_option*
	(make <chunks-option>
		#:main-weight-proc (lambda (t f l) (* (+ t l) f))
		#:supp-weight-proc (lambda (t f l) (* (+ t l) (- 2 f)))
		#:verb-limit 2
		#:noun-limit 3
		#:adj-limit 5
		#:adv-limit 3
	)
)


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
(define (microplanning seq-link utterance-type option anaphora)
	(define all-sets '())
	
	(define (wrap-setlink atoms ut)
		; add additional link base on utterance type
		(SetLink (get-utterance-link ut atoms) atoms)
	)
	(define (finalize set)
		(define chunks (get-chunks set))

		; XXX after inserting anaphora, a chunk might no longer be say-able
		(if anaphora
			(set! chunks (insert-anaphora chunks (map get-sentence-forms (get-utterance-types set))))
		)
		
		(map wrap-setlink chunks (get-utterance-types set))
	)

	(cond ((equal? 'SequentialAndLink (cog-type seq-link))
		; initialize the sentence forms as needed
		(microplanning-init)

		(set! all-sets (make-sentence-chunks (cog-outgoing-set seq-link) utterance-type option))

		(cond ((not (null? all-sets))
			(map finalize all-sets)
		      )
		      (else
			#f
		      )
		)
	      )
	      (else
		(scm-error 'wrong-type-arg "microplanning" "Wrong type (expecting SequentialAndLink): ~A" (list seq-link) (list seq-link))
	      )
	)
)

; -----------------------------------------------------------------------
; make-sentence-chunks -- The main helper function for handling a microplanning request
;
; Calls make-sentence repeatedly until all informations have been spoken, or
; no longer say-able.  Accepts a list of links 'atoms-set' from within the original
; SequentialAndLink, and the utterance-type.  Returns a list of different ways
; chunks can form, each set of chunks contained within an <chunks-set> object.
;
(define (make-sentence-chunks atoms-set utterance-type option)
	(define all-sets '())

	(define (recursive-helper curr-unused curr-chunks curr-uts)
		(define ut (list utterance-type))
		
		; helper function for branching into different utterance type
		(define (sub-helper ut)
			(define new-chunk (make-sentence curr-unused atoms-set ut option))
			(cond ; has a new chunk, continue to make more chunk with the rest of the atoms
			      ((not (null? new-chunk))
				; TODO keep some of the atoms (those that do not satisfy sentence forms) for later use?
				(recursive-helper
					(lset-difference equal? curr-unused new-chunk)
					(cons new-chunk curr-chunks) 
					(cons ut curr-uts)
				)
			      )
			      ; unable to form more chunks, store the created chunks (if any) & their corresponding utterance-type
			      ((not (null? curr-chunks))
				(set! all-sets
					(cons
						(make <chunks-set>
							#:chunks (reverse curr-chunks)
							#:utterance-types (reverse curr-uts)
							#:leftover-count (length curr-unused)
						)
						all-sets
					)
				)
			      )
			)		
		)
		
		; if not the first sentence and has "interrogative" utterance type, allow "declarative"
		(if (and (not (null? curr-chunks)) (string=? "interrogative" utterance-type))
			(set! ut (list "interrogative" "declarative"))
		)

		(cond ; use the sub-helper to keep calling make sentence until all atoms are used, on all allowed utterance type
		      ((not (null? curr-unused))
			(for-each sub-helper ut)
		      )
		      ; finished all atoms, store the created chunks & their corresponding utterance-type
		      (else
			(set! all-sets
				(cons
					(make <chunks-set>
						#:chunks (reverse curr-chunks)
						#:utterance-types (reverse curr-uts)
						#:leftover-count 0
					)
					all-sets
				)
			)
		      )
		)
	)

	; loop make-sentence on remaining atoms
	(recursive-helper atoms-set '() '())

	(cond ((not (null? all-sets))
		(receive (complete-sets incomplete-sets)
			(partition (lambda (cs) (= (get-leftover-count cs) 0)) all-sets)
			
			; remove sets in incomplete-sets which is a subset of a set in complete-sets
			(set! incomplete-sets (remove (lambda (is) (any is-subset? (circular-list is) complete-sets)) incomplete-sets))
			
			; incomplete chunks set (those which did not say everything) are sorted by how many atoms leftover
			(sort! incomplete-sets less-leftover?)
			
			; sort the complete sets bases on how many chunks (sentences) in the set
			(sort! complete-sets less-chunks?)
			
			; sort sets with same amount of sentences bases on variation
			(letrec ((sort-by-variation
					(lambda (sets)
						(if (null? sets)
							'()
							(receive (this next)
								; split by finding all whose # of sentences is the same as the first in the list
								(span (lambda (c) (= (get-length c) (get-length (car sets)))) sets)
								(append
									(sort this less-variation?)
									(sort-by-variation next)
								)
							)
						)
					)
				))
				(append (sort-by-variation complete-sets) incomplete-sets)
			)
		)
	      )
	      (else '())
	)
)

; -----------------------------------------------------------------------
; make-sentence -- Create a single sentence
;
; Greedily add new atoms (using some heuristics) to form a new sentence.
; Accepts a list of un-said links, the complete set, and the utterance type.
;
(define (make-sentence atoms-unused atoms-set utterance-type option)
	; bunch of variables for storing the recursive looping results
	(define atoms-used (lset-difference equal? atoms-set atoms-unused)) ; the set of both failed and successful atoms
	(define atoms-failed '())	; the set of failed atoms
	(define chunk '())		; the set of successful atoms to be returned

	; main helper function for looping
	(define (recursive-helper atoms-to-try)
		(define result (check-chunk atoms-to-try utterance-type option)) ; the result of trying to say the atoms in a sentence
		(define atoms-not-tried (lset-difference equal? atoms-unused atoms-to-try)) ; the set of atoms not yet used

		(define (give-up-unadded-part)
			; atoms that did not work
			(define dead-set (lset-difference equal? atoms-to-try chunk))
			; atoms that worked
			(define good-set (lset-intersection equal? atoms-to-try chunk))

			(set! atoms-failed (append dead-set atoms-failed))
			(set! atoms-unused (lset-difference equal? atoms-unused dead-set))
			
			; try "saying" the previous working iteration again (if available) and re-choose a new atom (using different weight proc)
			(if (null? good-set)
				(if (not (null? atoms-unused))
					(recursive-helper (list (pick-atom atoms-unused atoms-used (get-main-weight-proc option) utterance-type)))
				)
				(recursive-helper good-set)
			)
		)
		(define (update-chunk)
			; add anything that has not been included in the chunk yet
			(set! chunk (lset-union equal? atoms-to-try chunk))
			(set! atoms-unused atoms-not-tried)
		)

		; temporary variables not all conditions needed
		(define temp-var1 '())
		(define temp-var2 '())
		(define temp-differences '())
		
		(cond ; not say-able
		      ((= result *microplanning_not_sayable*)
			; could be the atoms cannot be said unless bringing in additional atoms (such as "and/or/that")
			; try to add more (up to 3 different links)
			(cond ((<= (length atoms-to-try) 3)
				; look to see if the newest link has any node that is solo (ie. appear only once in the current set)
				(set! temp-var1 (cog-get-all-nodes (car atoms-to-try)))
				(set! temp-var2 (append-map cog-get-all-nodes (cdr atoms-to-try)))
				(set! temp-differences (filter has-word-inst? (lset-difference equal? temp-var1 temp-var2)))

				(cond ((null? temp-differences)
					(give-up-unadded-part)
				      )
				      (else
					; find the first link in atoms-not-tried that contains one of the solo word
					(set! temp-var1
						(find (lambda (a)
							(find (lambda (w) (cog-has-node? a w)) temp-differences)
						     )
						     atoms-not-tried
						)
					)

					; if an atom with the solo word exists
					(if temp-var1
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
		      ((= result *microplanning_sayable*)
			; add the currently "say-able" stuff to our chunk
			(update-chunk)

			; only continue if there are more to say
			(if (not (null? atoms-unused))
				(recursive-helper
					(cons
						; weight is (time-weights + link-weights) * (2 - form-weights), ie. prefer links that do not
						; satisify a sentence form, meaning it cannot be "said" on its own.
						(pick-atom atoms-unused chunk (get-supp-weight-proc option) utterance-type)
						atoms-to-try
					)
				)
			)
		      )
		      ; too long/complex
		      (else
			(update-chunk)
		      )
		)
	)

	; the initial critera for choosing a starting point would be (time-weights + link-weights) * form-weights
	(recursive-helper (list (pick-atom atoms-unused atoms-used (get-main-weight-proc option) utterance-type)))

	; return the sentence chunk (reverse because we've been adding to the front)
	(reverse chunk)
)

; -----------------------------------------------------------------------
; get-sentence-forms -- Get the sentence forms base on utterance type
;
; Base on the utterance type, get the sentence forms defined for it.
;
(define (get-sentence-forms utterance-type)
	(cog-outgoing-set (car
		(cond ((string=? "declarative" utterance-type)
			(cog-chase-link 'InheritanceLink 'OrLink (ConceptNode "DeclarativeUtterance"))
		      )
		      ((string=? "interrogative" utterance-type)
			(cog-chase-link 'InheritanceLink 'OrLink (ConceptNode "InterrogativeUtterance"))
		      )
		      ((string=? "imperative" utterance-type)
			(cog-chase-link 'InheritanceLink 'OrLink (ConceptNode "ImperativeUtterance"))
		      )
		      ((string=? "interjective" utterance-type)
			'()
		      )
		)
	))
)

; -----------------------------------------------------------------------
; get-utterance-link -- Get the additional link required for an utterance type
;
; Get the additional link required for SuReal to match to sentence of a
; specific utterance type.  To be inserted as part of the output SetLink.
;
(define (get-utterance-link utterance-type atoms)
	; needs the following to handle utterance type with multiple speech acts
	; by doing a hacky search for VariableNode in the sentence-form link
	(define favored-forms (get-sentence-forms utterance-type))
	(define (search-varnode)
		(define (helper atom)
			(and (match-sentence-forms atom favored-forms) (cog-has-atom-type? atom 'VariableNode))
		)
		
		(any helper atoms)
	)
	
	(cond ((string=? "declarative" utterance-type)
		(InheritanceLink (InterpretationNode "MicroplanningNewSentence") (ConceptNode "DeclarativeSpeechAct"))
	      )
	      ((string=? "interrogative" utterance-type)
		; TruthQuerySpeechAct will have no VariableNode on the main sentence-form link
		(if (search-varnode)
			(InheritanceLink (InterpretationNode "MicroplanningNewSentence") (ConceptNode "InterrogativeSpeechAct"))
			(InheritanceLink (InterpretationNode "MicroplanningNewSentence") (ConceptNode "TruthQuerySpeechAct"))
		)
	      )
	      ((string=? "imperative" utterance-type)
		(InheritanceLink (InterpretationNode "MicroplanningNewSentence") (ConceptNode "ImperativeSpeechAct"))
	      )
	      ((string=? "interjective" utterance-type)
		(InheritanceLink (InterpretationNode "MicroplanningNewSentence") (ConceptNode "InterjectiveSpeechAct"))
	      )
	)
)

; -----------------------------------------------------------------------
; pick-atom -- Pick the best atom using some weighting function
;
; Helper function that weights the atoms in 'atoms-list' against 'bases-list'
; using function 'comb-proc', and choose the one with the highest weight.
;
; 'comb-proc' should be a function that takes in param "time", "form", "link"
; in this order
;
(define (pick-atom atoms-list bases-list comb-proc utterance-type)
	(define favored-forms (get-sentence-forms utterance-type))
	
	; helper function that calculate the weights of each atoms in 'choices' using 'comb-proc'
	(define (calc-weights choices bases comb-proc)
		(define choices-len (length choices))

		; helper function: get all nodes for a atom, get all nodes in 'bases', count how many common nodes
		(define (link-count atom)
			(define atom-nodes (cog-get-all-nodes atom))
			(define used-nodes (delete-duplicates (append-map cog-get-all-nodes bases)))
			(length (lset-intersection equal? atom-nodes used-nodes))
		)

		; initialize weight bases on "time ordering"
		(define time-weights (list-tabulate choices-len (lambda (i) (- choices-len i))))
		; initialize weight (0, 1) bases on whether the atom satisfy a sentence form
		(define form-weights (map (lambda (atom) (if (match-sentence-forms atom favored-forms) 1 0)) choices))
		; initialize weight bases on linkage to other atoms in 'bases'
		(define link-weights (map link-count choices))
		
		; create a combined weight using comb-proc
		(map comb-proc time-weights form-weights link-weights)
	)

	(define weights (calc-weights atoms-list bases-list comb-proc))
	(define assoc-list (sort (map cons atoms-list weights) (lambda (x y) (> (cdr x) (cdr y)))))
	(caar assoc-list)
)

; -----------------------------------------------------------------------
; check-chunk -- Check if a chunk is "say-able"
;
; Call Surface Realization pipeline to see if a chunk of 'atoms' are
; "say-able" for a specific 'utterance-type', and if so, determines whether
; the sentence is too long or complex.
;
(define (check-chunk atoms utterance-type option)
	(define (get-pos n)
		(define wi (r2l-get-word-inst n))
		(if (null? wi)
			"_"
			(cog-name (car (word-inst-get-pos wi)))
		)
	)
	(define (sort-n-count l)
		(define sorted-list (sort l string<?))
		(define (count-helper curr-subset curr-pos curr-count)
			(cond ((null? curr-subset)
				(cons (cons curr-pos curr-count) '())
			      )
			      ((not (string=? (car curr-subset) curr-pos))
				(cons (cons curr-pos curr-count) (count-helper (cdr curr-subset) (car curr-subset) 1))
			      )
			      (else
				(count-helper (cdr curr-subset) curr-pos (+ 1 curr-count))
			      )
			)
		)
		(if (not (null? sorted-list))
			(count-helper (cdr sorted-list) (car sorted-list) 1)
			'()
		)
	)

	; XXX optimization needed?
	(define all-nodes (delete-duplicates (append-map cog-get-all-nodes atoms)))
	(define pos-alist (sort-n-count (map get-pos all-nodes)))
	
	(define (pos-check? al)
		(define pos (car al))
		(define count (cdr al))
		
		(cond ; prefer max 2 verbs per sentence
		      ((and (string=? "verb" pos) (>= count (get-verb-limit option)))
			#f
		      )
		      ; prefer max 3 nouns per sentence
		      ((and (string=? "noun" pos) (>= count (get-noun-limit option)))
			#f
		      )
		      ; prefer max 5 adjectives
		      ((and (string=? "adj" pos) (>= count (get-adj-limit option)))
			#f
		      )
		      ; prefer max 3 adverbs
		      ((and (string=? "adv" pos) (>= count (get-adv-limit option)))
			#f
		      )
		      (else
			#t
		      )
		 )
	)

	(define pos-result (every pos-check? pos-alist))

	(define temp-set-link (SetLink (get-utterance-link utterance-type atoms) atoms))

	; do something with SuReal to see if all atoms can be included in a sentence
	(define say-able #f)
	(receive (sentences weights) (sureal temp-set-link)
		(if (any (lambda (x) (>= x 0)) weights)
			(set! say-able #t)
		)
	)
	
	; remove the temporary SetLink
	(cog-delete temp-set-link)

	(cond 
	      ; not long/complex but sayable
	      ((and pos-result say-able) *microplanning_sayable*)
	      ; long/complex but sayable
	      ((and (not pos-result) say-able) *microplanning_too_long*)
	      ; not sayable
	      (else *microplanning_not_sayable*)
	)
)


;;;;;;;; brain organizing stuff 

; if not long enough
; add external links that share a node? how to determine what to include?
; add the next highest weight atom?
;    prefer atoms with low form-weights & high link weights (with the new chunk, not the old stuff said)?


; some atoms used can be leave out of used to be reused later?
; eg. atoms that do not satisfy a sentence form (like adjectives)?


