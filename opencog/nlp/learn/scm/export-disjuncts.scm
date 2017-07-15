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
(use-modules (opencog matrix))
(use-modules (opencog sheaf))

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
; Given left and right strings, return the connector letters.
(define (get-cnr-name LSTR RSTR)
)

;  ---------------------------------------------------------------------

(define cnr-to-left (ConnectorDir "-"))

(define (cset-to-lg SECTION)
"
  cset-to-lg - SECTION should be a SectionLink
"
	; The string name of the germ of the section.
	(define germ (cog-name (gar SECTION)))
	(define (connector-to-lg CONNECTOR)
		(define cnr (cog-name (gar CONNECTOR)))
		(define dir (gdr CONNECTOR))

		(if (equal? dir cnr-to-left)
			(get-cnr-name cnr germ)
			(get-cnr-name germ cnr)
		)
	)

	(map connector-to-lg (cog-outgoing-set (gdr SECTION)))
)

;  ---------------------------------------------------------------------

(define (export-them)
	(define psa (make-pseudo-cset-api))

	; Get from SQL
	(psa 'fetch-pairs)

	(define all-csets (psa 'all-pairs))

	(map cset-to-lg all-csets)
)
;  ---------------------------------------------------------------------
