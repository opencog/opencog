;
; mst-parser.scm
;
; Minimum Spanning Tree parser.
;
; Copyright (c) 2014 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; The scripts below implement a simple spanning-tree parser. The goal
; of this parser is to learn a base set of link-grammar disjuncts.
;
; Input to this should be a single sentence. It is presumed, as
; background, that a large number of word-pairs with associated mutual
; information is already avaialable from the atomspace (obtained
; previously).
;
; The algorithm implemented is a basic minimum spanning tree algorithm.
; Initially, a graph clique is constructed from the words in the
; sentence, with every word connected to every other word. The mutual
; information betweeen wrd-pairs is used to determine graph edge lengths.
; The spanning tree is then obtained. Finally, disjuncts are created from
; the resulting parse, by looking at how each word is attached to the
; other words.  The disjuncts are then recorded.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))

; ---------------------------------------------------------------------
;
; Tokenize the text: take the input sentence, and return a list of the
; words in the sentence.  Words are always separated by white-space, so
; this is easy. This also makes a vague attempt to also separate commas,
; periods, and other punctuation.  Returned is a list of words.
;
(define (tokenize-text plain-text)
	(define prefix "(")
	(define suffix ".,)")
	(define prefix-list (string->list prefix))
	(define suffix-list (string->list suffix))

	(define (strip-sufli word sufli)
      (if (null? sufli)
			(list word)
			(let* ((punct (car sufli))
					(loc (string-rindex word punct)))
				(if loc 
					(append 
						(strip-suffix (substring word 0 loc))
						(list (string punct))
					)
					(strip-sufli word (cdr sufli))
				)
			)
		)
	)
	(define (strip-suffix word) (strip-sufli word suffix-list))
		
	(let* ((word-list (string-split plain-text #\ )))
		(concatenate (map strip-suffix word-list))
	)
)

; ---------------------------------------------------------------------

(define (mst-parse-text plain-text)
)

