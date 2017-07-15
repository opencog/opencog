;
; make-disjuncts.scm
;
; Compute the disjuncts, obtained from an MST parse of a sentence.
;
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
; In either case, once can work "backwards", and obtain the effective
; disjunct on each word, that would have lead to the given MST parse.
; For each word, this disjunct is just the collection of the other words
; that it is connected to. It is the unit-distance section of a sheaf.
;
; All the hard work is done in the `sheaf` module. This is just a very
; slim wrapper to parse the text, and update the number of times the
; disjunct has been observed.
; ---------------------------------------------------------------------

(use-modules (srfi srfi-1))
(use-modules (opencog sheaf))

;  ---------------------------------------------------------------------

; Return #t if the section is bigger than what the current postgres
; backend can store. Currently, the limit is atoms with at most 330
; atoms in the outgoing set.
;
; This typically occurs when the MST parser is fed a long string of
; punctuation, or a table of some kind, or other strings that are not
; actual sentences.
(define (is-oversize? SECTION)
	(< 330 (cog-arity (gdr SECTION)))
)

(define-public (observe-mst plain-text)
"
  observe-mst -- update pseduo-disjunct counts by observing raw text.

  This is the second part of the learning algo: simply count how
  often pseudo-disjuncts show up.
"
	; The count-one-atom function fetches from the SQL database,
	; increments the count by one, and stores the result back
	(for-each
		(lambda (dj) (if (not (is-oversize? dj)) (count-one-atom dj)))
		(make-sections (mst-parse-text plain-text))
	)
)
;  ---------------------------------------------------------------------
