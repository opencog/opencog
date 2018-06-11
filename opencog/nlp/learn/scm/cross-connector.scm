;
; cross-connector.scm
;
; Representing words as vectors crossing over into individual connectors.
;
; Copyright (c) 2017 Linas Vepstas
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; This file provide the "matrix-object" API that allows words to be
; treated as vectors ranging over individual connectors contained inside
; thier disjuncts. It "crosses" over, in a certain sideways-intersection
; sense, between words, and the other words inside of thier connectors.
;
; A much simpler example of a word-vector can be found in
; `pseudo-csets.scm`. That's a better place to start. This file
; implements a more complex idea.
;
; Consider, for example, the section
;
;    (Section
;       (WordNode "playing")
;       (ConnectorSeq
;          (Connector
;             (WordNode "level")
;             (LgConnDirNode "-"))
;          (Connector
;             (WordNode "field")
;             (LgConnDirNode "+"))))
;
; There are two pairs of interest, here: (playing, level) and (playing,
; field).  The code in this file provides a matrix-object API for
; accessing either one of these pairs, or, more specifically, pairs,
; where the 1st (left) member is the Word/WordClass of the section,
; and the 2nd (right) member is the K'th element in a N-connector
; disjunct on that section.  Sections that do not have exactly N
; connectors are excluded from consideration.
;
; ---------------------------------------------------------------------
;
(use-modules (srfi srfi-1))
(use-modules (opencog))
(use-modules (opencog persist))
(use-modules (opencog matrix))

(define-public (make-cross-section-api K N)
"
  make-cross-api K N -- API for cross-section word-pairs.

  The 1st (left) member is the Word/WordClass of the section, while
  the 2nd (right) member is the K'th element in a N-connector disjunct
  on that section.  Sections that do not have exactly N connectors are
  excluded from consideration.

  A more detailed description is at the top of this file.
"
	(let ((all-csets '())
			(Kay K)   ; Cache K and N for later use.
			(Nnn N))

		; Get the observational count on Section SECT
		(define (get-count SECT) (cog-tv-count (cog-tv SECT)))

		(define any-left (AnyNode "cset-word"))
		(define any-right (AnyNode
			(format #f "cset-cross-connector-~D-~D" K N)))

		(define (get-left-type) 'WordNode)
		(define (get-right-type) 'WordNode)
		(define (get-pair-type) 'Section)

xxxxxxxxx
		; Getting the pair is non-trivial....
		(define (get-pair PAIR) PAIR)

		; Getting the count is .... XXX fixme
		(define (get-pair-count PAIR) (get-count PAIR))

		(define (get-left-wildcard DJ)
			(ListLink any-left DJ))

		(define (get-right-wildcard WORD)
			(ListLink WORD any-right))

		(define (get-wild-wild)
			(ListLink any-left any-right))

		; Fetch (from the database) all pseudo-csets
		(define (fetch-pseudo-csets)
			(define start-time (current-time))
			; marginals are located on any-left, any-right
			(fetch-incoming-set any-left)
			(fetch-incoming-set any-right)
			(load-atoms-of-type 'Section)
			(format #t "Elapsed time to load csets: ~A secs\n"
				(- (current-time) start-time)))

		; Methods on the object
		(lambda (message . args)
			(apply (case message
				((name) (lambda () "Word, Cross-section-Word Pairs"))
				((id)   (lambda () "cross-section"))
				((left-type) get-left-type)
				((right-type) get-right-type)
				((pair-type) get-pair-type)
				((pair-count) get-pair-count)
				((item-pair) get-pair)
				((make-pair) get-pair)
				((left-wildcard) get-left-wildcard)
				((right-wildcard) get-right-wildcard)
				((wild-wild) get-wild-wild)
				((fetch-pairs) fetch-pseudo-csets)
				((provides) (lambda (symb) #f))
				((filters?) (lambda () #f))
				(else (error "Bad method call on cross-section:" message)))
			args)))
)

; ---------------------------------------------------------------------
