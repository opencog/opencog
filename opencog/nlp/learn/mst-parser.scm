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
	; Prefix and suffix lists taken from the link-grammar ANY
	; language 4.0.affix file
	(define prefix "({[<«〈（〔《【［『「``„“‘'''…...¿¡$£₤€¤₳฿₡₢₠₫৳ƒ₣₲₴₭₺ℳ₥₦₧₱₰₹₨₪﷼₸₮₩¥៛호점†††‡§¶©®℗№#")
	(define suffix ")}]>»〉）〕》】］』」’'%,.。:;?!‽؟？！…”–‐、～¢₵™℠")
	(define prefix-list (string->list prefix))
	(define suffix-list (string->list suffix))

	; Tail-recursive helper function. Strip off any prefixes appearing
	; in the list prefli, and return a list of the stripped prefixes,
	; and the remaining word.
	(define (strip-prefli word prefli)
      (if (null? prefli)
			(list word)  ; prefix list is empty, return the word.
			(let* ((punct (car prefli))
					(head (string-ref word 0)))
				(if (eq? punct head) ; it there's a match, then split
					(append 
						(list (string punct))
						(strip-prefix (substring word 1)) ; loop again.
					)
					(strip-prefli word (cdr prefli)) ; if no match, recurse.
				)
			)
		)
	)
	; Main entry point for the recursive prefix splitter
	(define (strip-prefix word) (strip-prefli word prefix-list))

	; Tail-recursive helper function. Strip off any suffixes appearing
	; in the list sufli, and return a list of the stripped prefixes,
	; the remaining word, and the stripped suffixes.
	(define (strip-sufli word sufli)
      (if (null? sufli)
			(strip-prefix word)
			(let* ((punct (car sufli))
					(len (- (string-length word) 1))
					(tail (string-ref word len)))
				(if (eq? punct tail)
					(append 
						(strip-affix (substring word 0 len))
						(list (string punct))
					)
					(strip-sufli word (cdr sufli))
				)
			)
		)
	)
	; Main entry point for the recursive splitter
	(define (strip-affix word) (strip-sufli word suffix-list))

	(let* ((word-list (string-split plain-text #\ )))
		(concatenate (map strip-affix word-list))
	)
)

; ---------------------------------------------------------------------

(define (mst-parse-text plain-text)
)

