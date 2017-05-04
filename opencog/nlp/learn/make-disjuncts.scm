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
; link-names.  This prepends the letter "M" to all names, so that
; all MST link-names start with this letter.
; Example:  0 --> MA, 1 --> MB
(define (number->tag num)

	; Convert number to a list of letters.
	(define (number->letters num)
		(define letters "ABCDEFGHIJKLMNOPQRSTUVWXYZ")
		(unfold-right negative?
			(lambda (i) (string-ref letters (remainder i 26)))
			(lambda (i) (- (quotient i 26) 1))
			num))

	(list->string (cons #\M (number->letters num)))
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
;          MSTLinkNode "MXYZ"
;          ListLink
;              WordNode "foo"
;              WordNode "bar"
;
; such that the "MXYZ" value is used only for this particular word-pair.
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
