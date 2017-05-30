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
(use-modules (opencog) (opencog persist))

; ---------------------------------------------------------------------
; Define locations where statistics will be stored.

; These are used to store/maintain counts for clique-pair counting.
; See `link-pipeline.scm` for usage.
(define pair-pred (PredicateNode "*-Sentence Word Pair-*"))
(define pair-dist (SchemaNode "*-Pair Distance-*"))

; get-count ATOM - return the raw observational count on ATOM.
(define-public (get-count ATOM) (cog-tv-count (cog-tv ATOM)))

; set-count ATOM CNT - Set the raw observational count on ATOM.
(define (set-count ATOM CNT) (cog-set-tv! ATOM (cog-new-ctv 0 0 CNT)))

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

(define-public (get-all-words)
"
  get-all-words - return a list holding all of the observed words
  This does NOT fetch the words from the backing store.
"
	(cog-get-atoms 'WordNode)
)

; ---------------------------------------------------------------------
