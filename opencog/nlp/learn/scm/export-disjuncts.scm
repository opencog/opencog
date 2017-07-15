;
; export-disjuncts.scm
;
; Export disjuncts from the atomspace into a dattabase that can be
; used by the Link-Grammar parser.
;
; Copyright (c) 2015 Rohit Shinde
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; After a collection of disjuncts has been observed by the MST pipeline,
; the can be exported to the link Grammar parser, where they can be used
; to parse sentences.
;
; In either case, once can work "backwards", and obtain the efective
; disjunct on each word, that would have lead to the given MST parse.
; The scripts in this file compute the disjunct.
;
; Currently an experimental hack job. Needs the guile-dbi interfaces,
; in order to write the SQL files.
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (dbi dbi))  ; The guile-dbi interface to SQLite3
(use-modules (opencog))
(use-modules (opencog matrix))
(use-modules (opencog sheaf))

; ---------------------------------------------------------------------
; Return a caching version of AFUNC. Here, AFUNC is a function that
; takes a single atom as an argument, and returns some object
; associated with that atom.
;
; This returns a function that returns the same values that AFUNC would
; return, for the same argument; but if a cached value is available,
; then return just that.  In order for the cache to be valid, the AFUNC
; must be side-effect-free.
;
(define (make-afunc-cache AFUNC)

	; Define the local hash table we will use.
	(define cache (make-hash-table))

	; Guile needs help computing the hash of an atom.
	(define (atom-hash ATOM SZ) (modulo (cog-handle ATOM) SZ))
	(define (atom-assoc ATOM ALIST)
		(find (lambda (pr) (equal? ATOM (car pr))) ALIST))

	(lambda (ITEM)
		(define val (hashx-ref atom-hash atom-assoc cache ITEM))
		(if val val
			(let ((fv (AFUNC ITEM)))
				(hashx-set! atom-hash atom-assoc cache ITEM fv)
				fv)))
)

; ---------------------------------------------------------------------
; Convert an integer into a string of letters. Useful for creating
; link-names.  This prepends the letter "T" to all names, so that
; all MST link-names start with this letter.
; Example:  0 --> TA, 1 --> TB
(define (number->tag num)

	; Convert number to a list of letters.
	(define (number->letters num)
		(define letters "ABCDEFGHIJKLMNOPQRSTUVWXYZ")
		(unfold-right negative?
			(lambda (i) (string-ref letters (remainder i 26)))
			(lambda (i) (- (quotient i 26) 1))
			num))

	(list->string (cons #\T (number->letters num)))
)

;  ---------------------------------------------------------------------
;
; Given a word-pair atom, return a synthetic link name
; The link names are issued in serial order, first-come, first-served.
;
(define get-cnr-name
	(let ((cnt 0))

		; Notice that the lambda does not actually depend on the
		; word-pair. It just issues a new string.  The function
		; cache is what is able to detect and re-emit a previously
		; issued link name.
		(make-afunc-cache
			(lambda (WORD-PAIR)
				(set! cnt (+ cnt 1))
				(number->tag cnt))))
)

;  ---------------------------------------------------------------------

(define cnr-to-left (ConnectorDir "-"))

(define (cset-to-lg-dj SECTION)
"
  cset-to-lg-dj - SECTION should be a SectionLink
  Return a link-grammar compatible disjunct string.
"
	; The germ of the section.
	(define germ (gar SECTION))

	; Get a link-name identifying this word-pair.
	(define (connector-to-lg-link CONNECTOR)
		(define cnr (gar CONNECTOR))
		(define dir (gdr CONNECTOR))

		(if (equal? dir cnr-to-left)
			(get-cnr-name (ListLink cnr germ))
			(get-cnr-name (ListLink germ cnr))
		)
	)

	; Get a connector, by concatenating the link name with the direction.
	(define (connector-to-lg-cnr CONNECTOR)
		(string-append
			(connector-to-lg-link CONNECTOR)
			(cog-name (gdr CONNECTOR))))

	; A list of connnectors, in the proper connector order.
	(define cnrs (map connector-to-lg-cnr (cog-outgoing-set (gdr SECTION))))

	; Create a single string of the connectors, in order.
	(fold
		(lambda (cnr dj) (if dj (string-append dj " & " cnr) cnr))
		#f cnrs)
)

;  ---------------------------------------------------------------------

(define (export-them)
	(define psa (make-pseudo-cset-api))

	; Get from SQL
	(psa 'fetch-pairs)

	(define all-csets (psa 'all-pairs))

	(map cset-to-lg-dj all-csets)
)
;  ---------------------------------------------------------------------
