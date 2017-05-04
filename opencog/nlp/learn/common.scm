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

; get-pair-link returns a list of atoms of type LNK-TYPE that contains
; the given PRED and PAIR.  This is used to obtain the atom that holds
; item-pair statistics, given a list-link holding the actual pair.
;
; PAIR is usually a ListLink of word-pairs.
; PRED is usually (PredicateNode "*-Sentence Word Pair-*")
;                 or (SchemaNode "*-Pair Distance-*")
;                 or (LinkGrammarRelationshopNode "ANY")
; LNK-TYPE is usually EvaluationLink or ExecutationLink
;
; PAIR is the atom (ListLink) we are given.  We want to find the
; LNK-TYPE that contains it.  That LNK-TYPE should have
; PRED as its predicate type, and the ListLink in the expected
; location. That is, given ListLink, we are looking for
;
; The result of this search is either #f or the EvaluationLink

(define (get-pair-link LNK-TYPE PRED PAIR)

	(filter!
		(lambda (evl)
			(define oset (cog-outgoing-set evl))
			(and
				(equal? LNK-TYPE (cog-type evl))
				(equal? PRED (car oset))
				(equal? PAIR (cadr oset))))
		(cog-incoming-set PAIR))
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Random-tree parse word-pair count access routines.
;
; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
;
(define (get-any-pair PAIR)
	(get-pair-link 'EvaluationLink any-pair-pred PAIR)
)

; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; ---------------------------------------------------------------------
; Clique-based-counting word-pair access methods.
; ---------------------------------------------------------------------

; Given a ListLink holding a word-pair, return the corresponding
; link that holds the count for that word-pair.
;
(define (get-clique-pair PAIR)
	(get-pair-link 'EvaluationLink pair-pred PAIR)
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
