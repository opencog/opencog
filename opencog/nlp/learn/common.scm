;
; common.scm
;
; Common functions shared between multpile functional units.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))

; ---------------------------------------------------------------------
; Define locations where statistics will be stored.

(define freq-key (PredicateNode "*-FrequencyKey-*"))
(define mi-key (PredicateNode "*-Mutual Info Key-*"))

; These are used to store/maintain counts for clique-pair counting.
; See `link-pipeline.scm` for usage.
(define any-pair-pred (LinkGrammarRelationshipNode "ANY"))
(define pair-pred (PredicateNode "*-Sentence Word Pair-*"))
(define pair-dist (SchemaNode "*-Pair Distance-*"))

; get-count ATOM - return the raw observational count on ATOM.
(define-public (get-count ATOM) (cog-tv-count (cog-tv ATOM)))

; set-count ATOM CNT - Set the raw observational count on ATOM.
(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 0 0 CNT)))

; ----
; set-freq ATOM FREQ - set the frequency count on ATOM.
;
; FREQ is assumed to be some simple ratio, interpreted as a
; probability: i.e. 0.0 < FREQ <= 1.0.  The frequency and it's log_2
; are stored: the log is accessed thousands of times, and so it
; is worth caching it as a pre-computed value.
;
; Returns ATOM.
;
(define (set-freq ATOM FREQ)
	; 1.4426950408889634 is 1/0.6931471805599453 is 1/log 2
	(define ln2 (* -1.4426950408889634 (log FREQ)))
	(cog-set-value! ATOM freq-key (FloatValue FREQ ln2))
)

; ----
; get-logli ATOM - get the -log_2(frequency) on ATOM.
;
; The log will be in position 2 of the value.
; This will throw an exception if no value has been recorded
; for this atom.
(define (get-logli ATOM)
	(cadr (cog-value->list (cog-value ATOM freq-key)))
)

; ----
; set-mi ATOM MI - set the mutual information on ATOM.
;
; MI is assumed to be a scheme floating-point value, holding the
; mutual-information value appropriate for the ATOM.
;
; In essentially all cases, ATOM is actually an EvaluationLink that
; is holding the structural pattern to which the mutial information
; applied. Currently, this is almost always a word-pair.
;
; Returns ATOM.
;
(define (set-mi ATOM MI)
	(cog-set-value! ATOM mi-key (FloatValue MI))
)

; ----
; get-mi ATOM - get the mutual information on ATOM.
;
; Returns a floating-point value holding the mutual information
; for the ATOM.
;
; In essentially all cases, ATOM is actually an EvaluationLink that
; is holding the structural pattern to which the mutial information
; applied. Currently, this is almost always a word-pair.
;
(define (get-mi ATOM)
	(car (cog-value->list (cog-value ATOM mi-key)))
)

; ---------------------------------------------------------------------

(define (count-one-atom ATM)
"
  count-one-atom ATM -- increment the count by one on ATM, and
  update the SQL database to hold that count.

  This will also automatically fetch the previous count from
  the SQL database, so that counting will work correctly, when
  picking up from a previous point.

  Warning: this is NOT SAFE for distributed processing! That is
  because this does NOT grab the count from the database every time,
  so if some other process updates the database, this will miss that
  update.
"
	(define (incr-one atom)
		; If the atom doesn't yet have a count TV attached to it,
		; then its probably a freshly created atom. Go fetch it
		; from SQL. Otherwise, assume that what we've got here,
		; in the atomspace, is the current copy.  This works if
		; there is only one process updating the counts.
		(if (not (cog-ctv? (cog-tv atom)))
			(fetch-atom atom)) ; get from SQL
		(cog-inc-count! atom 1) ; increment
	)
	(begin
		(incr-one ATM) ; increment the count on ATM
		(store-atom ATM)) ; save to SQL
)

; ---------------------------------------------------------------------
; Compute log liklihood of having observed a given atom.
;
; The liklihood and its log-base-2 will be stored under the key
; (Predicate "*-FrequencyKey-*"), with the first number being the
; frequency, which is just the atom's count value, dividing by the
; total number of times the atom has been observed.  The log liklihood
; is -log_2(frequency), and is stored as a convenience.
;
; This returns the atom that was provided, but now with the logli set.

(define (compute-atom-logli atom total)
	(set-freq atom (/ (get-count atom) total))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
;
(define-public (get-any-pair PAIR)
"
  get-any-pair PAIR -- Given a ListLink holding a word-pair, return
  the corresponding 'ANY' link-type link that holds the count for
  that word-pair.  Return the empty list, if there is no count for
  the word-pair.
"
	(cog-link 'EvaluationLink any-pair-pred PAIR)
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

(define-public (get-clique-pair PAIR)
"
  get-clique-pair PAIR -- Given a ListLink holding a word-pair, return
  the corresponding 'clique-counting' style link that holds the count
  for that word-pair. Return the empty list, if there is no count for
  the word-pair.
"
	(cog-link 'EvaluationLink pair-pred PAIR)
)
; ---------------------------------------------------------------------

(define-public (get-all-words)
"
  get-all-words - return a list holding all of the observed words
  This does NOT fetch the words from the backing store.
"
	(cog-get-atoms 'WordNode)
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
