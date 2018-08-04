;
; singletons.scm
;
; Stuff for working with singltons.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; Collection of rancid unfinished junk.

(use-modules (opencog))
(use-modules (opencog persist))

; ---------------------------------------------------------------------
; Count the total number of times that the atoms in the atom-list have
; been observed.  The observation-count for a single atom is stored in
; the 'count' value of its CountTruthValue. This routine just fetches
; those, and adds them up.
;
; The returned value is the total count.

(define-public (get-total-atom-count atom-list)
	;
	; A proceedural loop.
	;	(let ((cnt 0))
	;		(define (inc atom) (set! cnt (+ cnt (get-count atom))))
	;		(for-each inc atom-list)
	;		cnt
	;	)

	; textbook tail-recursive solution.
	(define (hlpr lst cnt)
		(if (null? lst) cnt
			(hlpr (cdr lst) (+ cnt (get-count (car lst))))))

	(hlpr atom-list 0)
)

; ---------------------------------------------------------------------
; set-freq ATOM FREQ - set the frequency count on ATOM.
;
; FREQ is assumed to be some simple ratio, interpreted as a
; probability: i.e. 0.0 < FREQ <= 1.0.  The frequency and it's log_2
; are stored: the log is accessed thousands of times, and so it
; is worth caching it as a pre-computed value.
;
; Returns ATOM.
;

(define freq-key (PredicateNode "*-FrequencyKey-*"))

(define (set-freq ATOM FREQ)
	; 1.4426950408889634 is 1/0.6931471805599453 is 1/log 2
	(define ln2 (* -1.4426950408889634 (log FREQ)))
	(cog-set-value! ATOM freq-key (FloatValue FREQ ln2))
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
; Compute the occurance logliklihoods for a list of atoms.
;
; This sums up the occurance-count over the entire list of atoms,
; and uses that as the normalization for the probability frequency
; for the individual atoms in the list. It then computes the log_2
; likelihood for each atom in the list, based on the total.
;
; As usual, the raw counts are obtained from the 'count' slot on a
; CountTruthValue, and the logli is stored as a value on the atom.
;
; This returns the atom-list, but now with the logli's set.

(define (compute-all-logli atom-list)
	(let ((total (get-total-atom-count atom-list)))
		(map
			(lambda (atom) (compute-atom-logli atom total))
			atom-list
		)
	)
)

; ---------------------------------------------------------------------
; Compute the occurance logliklihoods for all words.
;
; Load all word-nodes into the atomspace from SQL storage, if they
; are not already present.  This also loads the associated values.
;
; This returns the list of all word-nodes, with the logli's set.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(call-only-once fetch-all-words)
		(compute-all-logli (get-all-words))
	)
)

; ---------------------------------------------------------------------

(define-public (total-word-observations)
"
  total-word-observations -- return a total of the number of times
  any/all words were observed.  That is, compute and return N(*),
  as defined above, and in the diary.  This does NOT work from a
  cached value.  Also, this does NOT fetch atoms from the database!
"
   (get-total-atom-count (get-all-words))
)

; ---------------------------------------------------------------------

(define-public (get-sentence-count)
"
  get-sentence-count -- get the number of sentences observed.
  This does fetch the count from the database.

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (SentenceNode "ANY")))
)

(define-public (get-parse-count)
"
  get-parse-count -- get the number of parses observed.
  This does fetch the count from the database.

  This count is maintained by the link-pipeline code.
"
	(get-count (fetch-atom (ParseNode "ANY")))
)

(define-public (avg-sentence-length)
"
  avg-sentence-length -- get expected value for the number of words
  in the sentence.
"
	; Each word is counted once, in every parse.
	(/ (total-word-observations) (get-parse-count))
)

; ---------------------------------------------------------------------
