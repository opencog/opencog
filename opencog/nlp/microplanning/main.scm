; loading additional dependency
(load "sentence-forms.scm")
(load "helpers.scm")
(load "anaphora.scm")
(load "chunks-option.scm")
(load "chunks-set.scm")
(load "atomW.scm")

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
		#:form-limit 3
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
; or 'interjective', "option" is an <chunks-option> object, and "anaphora"
; can be #t or #f.
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
; SequentialAndLink, the utterance-type, and the option.  Returns a list of different
; ways chunks can form by varying utterance-types, each set of chunks contained
; within an <chunks-set> object.
;
(define (make-sentence-chunks atoms-list utterance-type option)
	; wrap each atom in a container to allow repeated atoms, and persistence time weights
	(define atomW-complete-set
		(map 
			(lambda (a t) (make <atomW> #:atom a #:time-weight t))
			atoms-list
			(iota (length atoms-list) (length atoms-list) -1)
		)
	)
	(define all-chunks-sets '())

	(define (recursive-helper atomW-unused curr-chunks curr-uts)
		(define ut (list utterance-type))
		
		; helper function for branching into different utterance type
		(define (sub-helper ut)
			(define new-atomW-chunk (make-sentence atomW-unused atomW-complete-set ut option))
			(cond ; has a new chunk, continue to make more chunk with the rest of the atoms
			      ((not (null? new-atomW-chunk))
				; TODO keep some of the atoms (those that do not satisfy sentence forms) for later use?
				(recursive-helper
					(lset-difference equal? atomW-unused new-atomW-chunk)
					(cons (map get-atom new-atomW-chunk) curr-chunks) 
					(cons ut curr-uts)
				)
			      )
			      ; unable to form more chunks, store the created chunks (if any) & their corresponding utterance-type
			      ((not (null? curr-chunks))
				(set! all-chunks-sets
					(cons
						(make <chunks-set>
							#:chunks (reverse curr-chunks)
							#:utterance-types (reverse curr-uts)
							#:leftover-count (length atomW-unused)
						)
						all-chunks-sets
					)
				)
			      )
			)		
		)
		
		; if not the first sentence and has "interrogative" utterance type, allow "declarative"
		(if (and (not (null? curr-chunks)) (string=? "interrogative" utterance-type))
			(set! ut (list "interrogative" "declarative"))
		)

		(cond ; use the sub-helper to keep calling make sentence until all atoms are used, branching on all allowed utterance type
		      ((not (null? atomW-unused))
			(for-each sub-helper ut)
		      )
		      ; finished all atoms, store the created chunks & their corresponding utterance-type
		      (else
			(set! all-chunks-sets
				(cons
					(make <chunks-set>
						#:chunks (reverse curr-chunks)
						#:utterance-types (reverse curr-uts)
						#:leftover-count 0
					)
					all-chunks-sets
				)
			)
		      )
		)
	)

	; loop make-sentence on remaining atoms
	(recursive-helper atomW-complete-set '() '())

	(cond ((not (null? all-chunks-sets))
		(receive (complete-sets incomplete-sets)
			(partition (lambda (cs) (= (get-leftover-count cs) 0)) all-chunks-sets)
			
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
; Accepts a list of un-said links, the complete set, the utterance type,
; and the chunking option.
;
(define (make-sentence atomW-unused atomW-complete-set utterance-type option)
	; bunch of variables for storing the recursive looping results
	(define atomW-used (lset-difference equal? atomW-complete-set atomW-unused)) ; the set of atoms said in previous sentences
	(define atomW-chunk '())	; the set of successful atoms to be returned

	; main helper function for looping
	(define (recursive-helper atomW-to-try)
		(define result (check-chunk (map get-atom atomW-to-try) utterance-type option)) ; the result of trying to say the atoms in a sentence
		(define atomW-not-tried (lset-difference equal? atomW-unused atomW-to-try)) ; the set of atoms not yet used

		(define (give-up-unadded-part)
			; atoms that did not work (not yet added to the chunk)
			(define dead-set (lset-difference equal? atomW-to-try atomW-chunk))
			; atoms that worked (already added to the chunk)
			(define good-set (lset-intersection equal? atomW-to-try atomW-chunk))

			; remove the stuff in dead-set from consideration
			(set! atomW-unused (lset-difference equal? atomW-unused dead-set))
			
			; try "saying" the previous working iteration again (if available)
			(if (null? good-set)
				(if (not (null? atomW-unused))
					(recursive-helper (list (pick-atomW atomW-unused atomW-used (get-main-weight-proc option) utterance-type)))
				)
				(recursive-helper good-set)
			)
		)
		(define (update-chunk)
			; add anything that has not been included in the chunk yet
			(set! atomW-chunk (lset-union equal? atomW-to-try atomW-chunk))
			(set! atomW-unused atomW-not-tried)
		)

		; temporary variables not all conditions needed
		(define temp-var1 '())
		(define temp-var2 '())
		(define temp-differences '())
		
		(cond ; not say-able
		      ((= result *microplanning_not_sayable*)
			; could be the atoms cannot be said unless bringing in additional atoms (such as "and/or/that")
			; try to add more (up to 3 different links)
			(cond ((<= (length atomW-to-try) 3)
				; look to see if the newest link has any node that is solo (ie. appear only once in the current set)
				(set! temp-var1 (cog-get-all-nodes (get-atom (car atomW-to-try))))
				(set! temp-var2 (append-map cog-get-all-nodes (map get-atom (cdr atomW-to-try))))
				(set! temp-differences (filter has-word-inst? (lset-difference equal? temp-var1 temp-var2)))

				(cond ((null? temp-differences)
					(give-up-unadded-part)
				      )
				      (else
					; find the first link in atomW-not-tried that contains one of the solo word
					; XXX could possibly allow choosing different link to generate multiple chunking result
					(set! temp-var1
						(find (lambda (a)
							(find (lambda (w) (cog-has-node? (get-atom a) w)) temp-differences)
						     )
						     atomW-not-tried
						)
					)

					; if an atom with the solo word exists
					(if temp-var1
						(recursive-helper (cons temp-var1 atomW-to-try))
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
			(if (not (null? atomW-unused))
				(recursive-helper
					(cons
						(pick-atomW atomW-unused atomW-chunk (get-supp-weight-proc option) utterance-type)
						atomW-to-try
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
	(recursive-helper (list (pick-atomW atomW-unused atomW-used (get-main-weight-proc option) utterance-type)))

	; return the sentence chunk (reverse because we've been adding to the front)
	(reverse atomW-chunk)
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
; pick-atomW -- Pick the best atom using some weighting function
;
; Helper function that weights the atomW in 'atomW-list' against bases,
; using function 'comb-proc', and choose the one with the highest weight.
;
; 'comb-proc' should be a function that takes in param "time", "form", "link"
; in this order
;
(define (pick-atomW atomW-list base-atomW-list comb-proc utterance-type)
	(define favored-forms (get-sentence-forms utterance-type))
	
	; helper function that calculate the weights of each atoms in 'choices' using 'comb-proc'
	(define (calc-weights choices bases comb-proc)
		(define unwrapped-choices (map get-atom choices))
		(define unwrapped-bases (map get-atom bases))

		; helper function: get all nodes for a atom, get all nodes in 'bases', count how many common nodes
		(define (link-count atom)
			(define atom-nodes (cog-get-all-nodes atom))
			(define bases-nodes (delete-duplicates (append-map cog-get-all-nodes unwrapped-bases)))
			(length (lset-intersection equal? atom-nodes bases-nodes))
		)

		; initialize weight bases on "time ordering"
		(define time-weights (map get-time-weight choices))
		; initialize weight (0, 1) bases on whether the atom satisfy a sentence form
		(define form-weights (map (lambda (atom) (if (match-sentence-forms atom favored-forms) 1 0)) unwrapped-choices))
		; initialize weight bases on linkage to other atoms in 'bases'
		(define link-weights (map link-count unwrapped-choices))
		
		; create a combined weight using comb-proc
		(map comb-proc time-weights form-weights link-weights)
	)

	(define weights (calc-weights atomW-list base-atomW-list comb-proc))
	(define assoc-list (sort (map cons atomW-list weights) (lambda (x y) (> (cdr x) (cdr y)))))
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
	(define favored-forms (get-sentence-forms utterance-type))
	(define ok-length
		(< (length (filter-map (lambda (l) (match-sentence-forms l favored-forms)) atoms)) (get-form-limit option))
	)

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
	      ((and ok-length say-able) *microplanning_sayable*)
	      ; long/complex but sayable
	      ((and (not ok-length) say-able) *microplanning_too_long*)
	      ; not sayable
	      (else *microplanning_not_sayable*)
	)
)


;;;;;;;; brain organizing stuff 

; if not long enough
; add external links that share a node? how to determine what to include?


; some atoms used can be leave out of used to be reused later?
; eg. atoms that do not satisfy a sentence form (like adjectives)?


