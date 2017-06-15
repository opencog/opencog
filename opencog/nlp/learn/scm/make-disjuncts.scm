;
; make-disjuncts.scm
;
; Compute the disjuncts, obtained from an MST parse of a sentence.
;
; Copyright (c) 2015 Rohit Shinde
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; After an sentence has been parsed with the MST parser, the links
; between words in the parse can be interpreted as Link Grammar links.
; There are two possible interpretations that can be given to these
; links: they are either "ANY" links, that connect between any words,
; or they are links capable of connecting ONLY those two words.
; In the later case, the link-name can be thought of as the
; concatenation of the two words it connects.
;
; In either case, once can work "backwards", and obtain the efective
; disjunct on each word, that would have lead to the given MST parse.
; The scripts in this file compute the disjunct.
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))

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

; ---------------------------------------------------------------------

(define-public (make-pseudo-disjuncts MST-PARSE)
"
  make-pseudo-disjuncts - create 'decoded' disjuncts.

  Given an MST parse of a sentence, return a list of 'decoded'
  disjuncts for each word in the sentence.

  It is the nature of MST parses that the links between the words
  have no labels: the links are of the 'any' type. We'd like to
  disover thier types, and we begin by creating pseudo-disjuncts.
  These resemble ordinary disjuncts, except that the connectors
  are replaced by the words that they connect to.

  So, for example, given the MST parse
     (mst-parse-text 'The game is played on a level playing field')
  the word 'playing' might get this connector set:

    (PseudoWordCset
       (WordNode \"playing\")
       (PseudoAnd
          (PseudoConnector
             (WordNode \"level\")
             (LgConnDirNode \"-\"))
          (PseudoConnector
             (WordNode \"field\")
             (LgConnDirNode \"+\"))))

  Grammatically-speaking, this is not a good connector, but it does
  show the general idea: that there was a link level<-->playing and
  a link playing<-->field.
"
	; Discard links with bad MI values; anything less than
	; -50 is bad. Heck, anything under minus ten...
	(define good-links (filter
		(lambda (mlink) (< -50 (mst-link-get-score mlink)))
		MST-PARSE))

	; Create a list of all of the words in the sentence.
	(define seq-list (delete-duplicates!
		(fold
			(lambda (mlnk lst)
				(cons (mst-link-get-left-numa mlnk)
					(cons (mst-link-get-right-numa mlnk) lst)))
			'()
			good-links)))

	; Return #t if word appears on the left side of mst-lnk
	(define (is-on-left-side? wrd mlnk)
		(equal? wrd (mst-link-get-left-atom mlnk)))
	(define (is-on-right-side? wrd mlnk)
		(equal? wrd (mst-link-get-right-atom mlnk)))

	; Given a word, and the mst-parse linkset, create a shorter
	; seq-list which holds only the words linked to the right.
	(define (mk-right-seqlist seq mparse)
		(define wrd (mst-numa-get-atom seq))
		(map mst-link-get-right-numa
			(filter
				(lambda (mlnk) (is-on-left-side? wrd mlnk))
				mparse)))

	(define (mk-left-seqlist seq mparse)
		(define wrd (mst-numa-get-atom seq))
		(map mst-link-get-left-numa
			(filter
				(lambda (mlnk) (is-on-right-side? wrd mlnk))
				mparse)))

	; Sort a seq-list into ascending order
	(define (sort-seqlist seq-list)
		(sort seq-list
			(lambda (sa sb)
				(< (mst-numa-get-index sa) (mst-numa-get-index sb)))))

	; Given a word, the the links, create a pseudo-disjunct
	(define (mk-pseudo seq mlist)
		(define lefts (sort-seqlist (mk-left-seqlist seq mlist)))
		(define rights (sort-seqlist (mk-right-seqlist seq mlist)))

		; Create a list of left-connectors
		(define left-cnc
			(map (lambda (sw)
					(PseudoConnector
						(mst-numa-get-atom sw)
						(LgConnDirNode "-")))
			lefts))

		(define right-cnc
			(map (lambda (sw)
					(PseudoConnector
						(mst-numa-get-atom sw)
						(LgConnDirNode "+")))
			rights))

		; return the connector-set
		(PseudoWordCset
			(mst-numa-get-atom seq)
			(PseudoAnd (append left-cnc right-cnc)))
	)

	(map
		(lambda (seq) (mk-pseudo seq MST-PARSE))
		seq-list)
)

;  ---------------------------------------------------------------------

(define-public (observe-mst plain-text)
"
  observe-mst -- update pseduo-disjunct counts by observing raw text.

  This is the second part of the learning algo: simply count how
  often pseudo-disjuncts show up.
"
	; The count-one-atom function fetches from the SQL database,
	; increments the count by one, and stores the result back
	(for-each
		(lambda (dj) (count-one-atom dj))
		(make-pseudo-disjuncts (mst-parse-text plain-text))
	)
)
;  ---------------------------------------------------------------------
