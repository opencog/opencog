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

; Implement the djb2 hash function.  STR is the string to hash,
; and MOD is the modulo to which the result should be taken.
; Example:
;  (string->hash "abc漢字def" 666667)
;
(define (string->hash STR MOD)
	(fold
		(lambda (n hash)
			(remainder (+ (* 33 hash) n) MOD))
		0
		(map char->integer (string->list STR)))
)

; ---------------------------------------------------------------------

; Create a link-label for a word-pair. The label is created by
; hashing together the two strings of the two words. The djb2
; hash function is used, the label is always consists of upper-case
; ASCII letters.  Yes, there may be hash collisions, but I think
; that (a) this is probably rare, (b) mostly doesn't matter.
; Its been adjusted so that the labels are always 10 chars or fewer
; in length, and the birthday-paradox chances of a hash collision
; are about 1 in a few million (sqrt (1<<43)).
;
(define (word-pair->label word-pair)
	; ash is `arithmetic-shift`, so maxlen is about 8 chars
	(define maxlen (- (ash 1 43) 3158534883327))
	(define hl (string->hash (cog-name (gar word-pair)) maxlen))
	(define hr (string->hash (cog-name (gdr word-pair)) maxlen))
	(define ph (remainder (+ (* 33 hl) hr) maxlen))
	(number->tag ph)
)

; ---------------------------------------------------------------------

; get-narrow-link -- create a "narrow" link between two words.
;
; Given a word-pair (ListLink (WordNode) (WordNode) this will
; return the corresponding stucture
;
;     EvaluationLink
;          MSTLinkNode "TXYZ"
;          ListLink
;              WordNode "foo"
;              WordNode "bar"
;
; such that the "TXYZ" value is used only for this particular word-pair.
; The label is computed by hashing.

(define (get-narrow-link word-pair)
	(Evaluation (MSTLink (word-pair->label word-pair)) word-pair)
)

; ---------------------------------------------------------------------

; Create "narrow" links for each link from an MST parse.
; No link is created if the MI of the MST parse is invalid.
; (i.e. if the MST parse had to use large negative MI's during
; the parse.)
;
; Example usage:  (make-narrow-links (mst-parse-text "this is a test"))
;
; XXX I think this code is actually useless. Which means that everything
; else up above is also useless. So maybe delete from beginging of file
; to about here.
;
(define (make-narrow-links mst-parse)
	(map
		; Create narrow links from MST-links
		(lambda (mst-lnk) (get-narrow-link (mst-link-get-wordpair mst-lnk)))
		(filter
			; discard links with bad MI values; anything less than
			; -50 is bad. Heck, anything under minus ten...
			(lambda (mlink) (< -50 (mst-link-get-mi mlink)))
			mst-parse))
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

    (LgWordCset
       (WordNode \"playing\")
       (LgAnd
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
		(lambda (mlink) (< -50 (mst-link-get-mi mlink)))
		MST-PARSE))

	; Create a list of all of the words in the sentence.
	(define seq-list (delete-duplicates!
		(fold
			(lambda (mlnk lst)
				(cons (mst-link-get-left-seq mlnk)
					(cons (mst-link-get-right-seq mlnk) lst)))
			'()
			good-links)))

	; Return #t if word appears on the left side of mst-lnk
	(define (is-on-left-side? wrd mlnk)
		(equal? wrd (mst-link-get-left-word mlnk)))
	(define (is-on-right-side? wrd mlnk)
		(equal? wrd (mst-link-get-right-word mlnk)))

	; Given a word, and the mst-parse linkset, create a shorter
	; seq-list which holds only the words linked to the right.
	(define (mk-right-seqlist seq mparse)
		(define wrd (mst-seq-get-word seq))
		(map mst-link-get-right-seq
			(filter
				(lambda (mlnk) (is-on-left-side? wrd mlnk))
				mparse)))

	(define (mk-left-seqlist seq mparse)
		(define wrd (mst-seq-get-word seq))
		(map mst-link-get-left-seq
			(filter
				(lambda (mlnk) (is-on-right-side? wrd mlnk))
				mparse)))

	; Sort a seq-list into ascending order
	(define (sort-seqlist seq-list)
		(sort seq-list
			(lambda (sa sb)
				(< (mst-seq-get-index sa) (mst-seq-get-index sb)))))

	; Given a word, the the links, create a pseudo-disjunct
	(define (mk-pseudo seq mlist)
		(define lefts (sort-seqlist (mk-left-seqlist seq mlist)))
		(define rights (sort-seqlist (mk-right-seqlist seq mlist)))

		; Create a list of left-connectors
		(define left-cnc
			(map (lambda (sw)
					(PseudoConnector
						(mst-seq-get-word sw)
						(LgConnDirNode "-")))
			lefts))

		(define right-cnc
			(map (lambda (sw)
					(PseudoConnector
						(mst-seq-get-word sw)
						(LgConnDirNode "+")))
			rights))

		; return the connector-set
		(LgWordCset
			(mst-seq-get-word seq)
			(LgAnd (append left-cnc right-cnc)))
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
