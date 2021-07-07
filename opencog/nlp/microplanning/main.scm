
(use-modules (ice-9 receive))  ; for receive, below
(use-modules (ice-9 optargs))  ; for define*-public
(use-modules (srfi srfi-1)
             (opencog)
             (opencog nlp)  ; need the atom types
             (opencog nlp oc)  ; need the atom types
             (opencog nlp relex2logic) ; helpers.scm uses this
             (opencog nlp sureal))

;
; loading additional dependency
(load "sentence-forms.scm")
(load "helpers.scm")
(load "anaphora-noun-item.scm")
(load "anaphora-nouns-list.scm")
(load "anaphora.scm")
(load "chunks-option.scm")
(load "chunks-set.scm")
(load "atomW.scm")


; =======================================================================
; Main interface
; =======================================================================

; -----------------------------------------------------------------------
; Because we use the case-lambda, here, instead of optargs,
; we cannot use define*-public, like we would like to. So the
; export is down below.  Oh well.
(define microplanning
	(case-lambda
		((sl ut) (microplanning-main sl ut *default_chunks_option* #t))
		((sl ut opt a) (microplanning-main sl ut opt a))
	)
)

(export microplanning)


(set-procedure-property! microplanning 'documentation
"
  microplanning -- The main microplanning interface

  A shortcut for calling microplanning without specifying all the
  arguments.
")

; =======================================================================
; Some constants
; =======================================================================

; TODO: Describe what these variables are for.
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

(export *default_chunks_option*)

; =======================================================================
; Main microplanning functions
; =======================================================================

; -----------------------------------------------------------------------
(define-public (microplanning-main seq-link utterance-type option anaphora)
"
  microplanning-main SEQ-LINK UTTERANCE-TYPE OPTION ANAPHORA

  The main microplanning function call.
  Accepts a SequentialAndLink containing a list of atoms to be spoken.
  UTTERANCE-TYPE is a string, either 'declarative', 'interrogative',
  'imperative' or 'interjective'.
  OPTION is a <chunks-option> object
  ANAPHORA can be #t or #f.
"
	; XXX FIXME utterance-type should be an atom, not a string!
	; viz (DefinedLinguisticConceptNode "DeclarativeSpeechAct") etc.
	; this would avoid a lot of string-matching/downcasing/appending
	; tomfoolery i.e. simplify the code.
	(define all-sets '())

	(define (finalize set)
		(define (wrap-setlink atoms ut)
			; add additional link base on utterance type
			(SetLink (get-utterance-link ut atoms) atoms)
		)

		(define new-set set)

		(if anaphora
			(set! new-set (insert-anaphora set))
		)
		(map wrap-setlink (get-chunks new-set) (get-utterance-types new-set))
	)

	; Initialize the sentence forms as needed
	(microplanning-init)

    ; Reset (clear) SuReal cache to assure it is empty before start calling sureal.
    ; Remark that SuReal Cache is not thread-safe. So it is not supposed to be used
    ; if the sureal queries are split into several threads. If you plan to do
    ; this (split sureal requests amongst multiple threads), just comment the
    ; following call and change the call to sureal below to use its non-cached version.
    ;
    ; It is not thread safe because in this version there is only one instance of
    ; the cache (reached via a singleton wrapper). So if two threads with two
    ; different Microplanner queries add stuff to the cache, one may lead to false
    ; hits in the other. In addition to this, the cache is reset (clear) just before
    ; the Microplanner query starts so if the second query reach the reset point
    ; before the first query ended, the cache will be reset during the lifetime
    ; of the first query, which may lead to false misses.
    ;
    ; A suggested approach to make it thread safe is having each Microplanner
    ; query to have its own cache instance.
	(reset-sureal-cache seq-link)

	(set! all-sets (make-sentence-chunks
		(cog-outgoing-set seq-link) utterance-type option))

	(if (nil? all-sets) #f (map finalize all-sets))
)

; -----------------------------------------------------------------------
(define (make-sentence-chunks atoms-list utterance-type option)
"
  make-sentence-chunks -- main helper function for microplanning

  Calls make-sentence repeatedly until all informations have been
  spoken, or no longer say-able.  Accepts a list of links 'atoms-set'
  from within the original ATOMS-LIST, the UTTERANCE-TYPE, and the
  OPTION.  Returns a list of different ways chunks can form by
  varying utterance-types, with each set of chunks contained
  within an <chunks-set> object.
"
	; Wrap each atom in a container to allow repeated atoms, and
	; persistence time weights
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

		; Helper function for branching into different utterance types.
		(define (sub-helper ut)
			(define new-atomW-chunk
				(make-sentence atomW-unused atomW-complete-set ut option))
			(cond
				; make-sentence made a new chunk; make more chunks
				; with the remaining atoms.
				((not (nil? new-atomW-chunk))
					; TODO Keep some of the atoms (those that do not
					; satisfy sentence forms) for later use?
					(recursive-helper
						(lset-difference equal? atomW-unused new-atomW-chunk)
						(cons (map get-atom new-atomW-chunk) curr-chunks)
						(cons ut curr-uts)
					)
				)
				; Unable to form more chunks, store the created chunks
				; (if any) & their corresponding utterance-type.
				((not (nil? curr-chunks))
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

		; If not the first sentence, and have "interrogative" utterance
		; type, allow "declarative"
		(if (and (not (nil? curr-chunks))
				(string=? "interrogative" utterance-type))
			(set! ut (list "interrogative" "declarative"))
		)

		(cond
			; Use the sub-helper to keep calling make-sentence until
			; all atoms are used, branching on all allowed utterance type.
			((not (nil? atomW-unused)) (for-each sub-helper ut))

			; Finished all atoms, store the created chunks & their
			; corresponding utterance-type.
			(else
				(set! all-chunks-sets
					(cons (make <chunks-set>
							#:chunks (reverse curr-chunks)
							#:utterance-types (reverse curr-uts)
							#:leftover-count 0)
						all-chunks-sets))
			)
		)
	)

	; loop make-sentence on remaining atoms
	(recursive-helper atomW-complete-set '() '())

	(cond
		((not (nil? all-chunks-sets))
			(receive (complete-sets incomplete-sets)
				(partition (lambda (cs) (= (get-leftover-count cs) 0)) all-chunks-sets)

				; Remove sets in incomplete-sets which are a subset
				; of a set in complete-sets
				(set! incomplete-sets
					(remove (lambda (is)
							(any is-subset? (circular-list is) complete-sets))
						incomplete-sets))

				; Incomplete chunks sets (those which did not say
				; everything) are sorted by how many atoms are leftover.
				(sort! incomplete-sets less-leftover?)

				; Sort the complete sets based on how many chunks
				; (sentences) are in the set.
				(sort! complete-sets less-chunks?)

				; Sort sets with same amount of sentences bases on
				; variation.
				(letrec ((sort-by-variation
						(lambda (sets)
							(if (nil? sets)
								'()
								(receive (this next)
									; Split by finding all whose # of sentences is
									; the same as the first in the list
									(span (lambda (c) (= (get-length c)
												(get-length (car sets)))) sets)
									(append
										(sort this less-variation?)
										(sort-by-variation next))
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
(define (make-sentence atomW-unused atomW-complete-set utterance-type option)
"
  make-sentence -- Create a single sentence

  Greedily add new atoms (using some heuristics) to form a new sentence.
  Accepts a list of un-said links, the complete set, the utterance type,
  and the chunking option.
"
	; Two variables for storing the recursive looping results.
	; --
	; The set of atoms said in previous sentences.
	(define atomW-used
		(lset-difference equal? atomW-complete-set atomW-unused))
	; The set of successful atoms to be returned.
	(define atomW-chunk '())

	; Main helper function for looping
	(define (recursive-helper atomW-to-try do-check)
		; The result of trying to say the atoms in a sentence.
		(define result
			; Return *microplanning_sayable* if no need to check.
			(if do-check
				(check-chunk (map get-atom atomW-to-try) utterance-type option)
				*microplanning_sayable*
			)
		)

		; The set of atoms not yet used.
		(define atomW-not-tried
			(lset-difference equal? atomW-unused atomW-to-try))

		; The set of atoms not yet added to the chunk.
		(define atomW-not-chunked
			(lset-difference equal? atomW-to-try atomW-chunk))

		(define (give-up-unadded-part)
			; Atoms that worked (already added to the chunk).
			(define good-set (lset-intersection equal? atomW-to-try atomW-chunk))

			; Remove the stuff in dead-set from consideration.
			(set! atomW-unused (lset-difference equal? atomW-unused atomW-not-chunked))

			; Try "saying" the previous working iteration again (if
			; available).
			(if (nil? good-set)
				(if (not (nil? atomW-unused))
					(recursive-helper (list
							(pick-atomW atomW-unused atomW-used
								(get-main-weight-proc option) utterance-type)) #t)
				)
				(recursive-helper good-set #f)
			)
		)
		(define (update-chunk)
			; Add anything that has not been included in the chunk yet.
			(set! atomW-chunk (lset-union equal? atomW-to-try atomW-chunk))
			(set! atomW-unused atomW-not-tried)
		)

		; temporary variables not all conditions needed
		(define temp-var1 '())
		(define temp-var2 '())
		(define temp-differences '())

		(cond ; not say-able
			((= result *microplanning_not_sayable*)
				; Could be the atoms cannot be said, without bringing in
				; additional atoms (such as "and/or/that").  Try to add
				; more (up to 3 different links).
				(cond ((<= (length atomW-not-chunked) 3)
						; Look to see if the newest link has any node that
						; is solo (i.e. appears only once in the current set).
						(set! temp-var1
							(cog-get-all-nodes (get-atom (car atomW-to-try))))
						(set! temp-var2
							(append-map cog-get-all-nodes
								(map get-atom (cdr atomW-to-try))))
						(set! temp-differences
							(filter has-word-inst?
								(lset-difference equal? temp-var1 temp-var2)))

						(cond
							((nil? temp-differences)
								(give-up-unadded-part))
							(else
								; Find the first link in atomW-not-tried that
								; contains one of the solo words.
								; XXX could possibly allow choosing different
								; link to generate multiple chunking result.
								(set! temp-var1
									(find (lambda (a)
										(find (lambda (w) (cog-has-node?
											(get-atom a) w)) temp-differences))
										atomW-not-tried))

								; If an atom with the solo word exists
								(if temp-var1
									(recursive-helper (cons temp-var1 atomW-to-try) #t)
									(give-up-unadded-part)
								)
							)
						)
					)
					; Give up on all the atoms not "say-able"
					(else
						(give-up-unadded-part)
					)
				)
			)
			; Not long/complex enough.
			((= result *microplanning_sayable*)
				; Add the currently "say-able" stuff to our chunk.
				(update-chunk)

				; Continue only if there is more to say.
				(if (not (nil? atomW-unused))
					(recursive-helper
						(cons
							(pick-atomW atomW-unused atomW-chunk
								(get-supp-weight-proc option) utterance-type)
							atomW-to-try
						)
						#t
					)
				)
			)
			; too long/complex
			(else
				(update-chunk)
			)
		)
	)

	; The initial critera for choosing a starting point would be
	; (time-weights + link-weights) * form-weights
	(recursive-helper (list (pick-atomW atomW-unused atomW-used
			(get-main-weight-proc option) utterance-type)) #t)

	; Return the sentence chunk (reverse because we've been adding
	; to the front).
	(reverse atomW-chunk)
)

; -----------------------------------------------------------------------
; get-sentence-forms -- Get the sentence forms base on utterance type
;
; Base on the utterance type, get the sentence forms defined for it.
;
(define (get-sentence-forms utterance-type)
	(cog-outgoing-set (car
		(cond
			((string=? "declarative" utterance-type)
				(cog-chase-link 'InheritanceLink 'OrLink
					(ConceptNode "DeclarativeUtterance")))

			((string=? "interrogative" utterance-type)
				(cog-chase-link 'InheritanceLink 'OrLink
					(ConceptNode "InterrogativeUtterance")))

			((string=? "imperative" utterance-type)
				(cog-chase-link 'InheritanceLink 'OrLink
					(ConceptNode "ImperativeUtterance")))
			((string=? "interjective" utterance-type) '())
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

	(cond
		((string=? "declarative" utterance-type)
			(InheritanceLink
				(InterpretationNode "MicroplanningNewSentence")
				(DefinedLinguisticConceptNode "DeclarativeSpeechAct")))

		((string=? "interrogative" utterance-type)
			; TruthQuerySpeechAct will have no VariableNode on the main
			; sentence-form link.
			(if (search-varnode)
				(InheritanceLink
					(InterpretationNode "MicroplanningNewSentence")
					(DefinedLinguisticConceptNode "InterrogativeSpeechAct"))
				(InheritanceLink
					(InterpretationNode "MicroplanningNewSentence")
					(DefinedLinguisticConceptNode "TruthQuerySpeechAct"))))

		((string=? "imperative" utterance-type)
			(InheritanceLink
				(InterpretationNode "MicroplanningNewSentence")
				(DefinedLinguisticConceptNode "ImperativeSpeechAct")))

		((string=? "interjective" utterance-type)
			(InheritanceLink
				(InterpretationNode "MicroplanningNewSentence")
				(DefinedLinguisticConceptNode "InterjectiveSpeechAct")))
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
    ;
    ; Remark that SuReal Cache is not thread-safe. So it is not supposed to be used
    ; if the sureal queries are split into several threads. If you plan to do
    ; this (split sureal requests among multiple threads), just comment the
    ; call to reset-sureal-cache above and change the line below to call 'sureal'
    ; instead of 'cached-sureal'
	(define say-able (not (nil? (cached-sureal temp-set-link))))

	; remove the temporary SetLink
	(cog-extract! temp-set-link)

	(cond
		; not long/complex but sayable
		((and ok-length say-able) *microplanning_sayable*)
		; long/complex but sayable
		((and (not ok-length) say-able) *microplanning_too_long*)
		; not sayable
		(else *microplanning_not_sayable*)
	)
)
