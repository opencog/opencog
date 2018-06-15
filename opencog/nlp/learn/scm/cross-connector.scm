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

(define-public (make-cross-section-api K DISJ)
"
  make-cross-api K PROTO -- API for cross-section word-pairs.

  The 1st (left) member is the Word/WordClass of the section, while
  the 2nd (right) member is the K'th element in the connector-sequence
  DISJ on that section.  Sections that do not have the same disjunct
  as DISJ (but differing only in position K) are excluded from
  consideration.

  A more detailed description is at the top of this file.
"
	(if (<= (cog-arity DISJ) K) (error "Bad offset K!"))

	(let ((all-csets '()))

		(define con-lst (cog-outgoing-set DISJ))
		(define start (take con-lst K))
		(define rest (drop con-lst K))
		(define conn (car rest))  ; the connector itself.
		(define end (cdr rest))
		(define conndir (gdr conn))  ; the connector direction

		(define any-left (AnyNode "cset-word"))
		(define any-right
			(ConnectorSeq
				start
				(Connector
					(AnyNode
						(format #f "cset-cross-connector-~D/~D"
							 K (cog-arity DISJ)))
					conndir)
				end))

		(define (get-left-type) 'WordNode)
		(define (get-right-type) 'WordNode)
		(define (get-pair-type) 'Section)

		; Get the observational count on Section SECT
		(define (get-count SECT) (cog-tv-count (cog-tv SECT)))

		; Both L-ATOM and R-ATOM are WordNodes (or WordClassNodes)
		(define (get-pair L-ATOM R-ATOM)
			(define con (cog-link 'Connector R-ATOM conndir))
			(if (null? con) '()
				(let ((seq (cog-link 'ConnectorSeq start con end)))
					(if (null? seq) '()
						(cog-link 'Section L-ATOM seq)))))

		(define (make-pair L-ATOM R-ATOM)
			(Section L-ATOM
				(ConnectorSeq start (Connector R-ATOM conndir) end)))

		; Return the Word or WordClass of the Section
		(define (get-left-element PAIR)
			(gar PAIR))

		; Return the K'th element in the disjunct
		; This does NO checking to make sure that the PAIR is valid.
		; I think that's OK, because this will always be called with
		; stars, which are a-priori always correct, so its a waste
		; of cpu time to check.
		(define (get-right-element PAIR)
			(gar (car (drop (cog-outgoing-set (gdr PAIR)) K))))

		; Get the count, if the pair exists.
		(define (get-pair-count L-ATOM R-ATOM)
			(define stats-atom (get-pair L-ATOM R-ATOM))
			(if (null? stats-atom) 0 (get-count stats-atom)))

		; Use ListLinks for the wild-cards, to avoid polluting
		; the space of Sections.  Is this a good idea? I dunno...
		;
		; This refuses to create bogus wild-cards for WORD.
		; I'm not sure if the extra overhead for this is worth
		; it, or if this will mess something up.
		(define (get-left-wildcard WORD)
			(define con (cog-link 'Connector WORD conndir))
			(if (null? con) '()
				(let ((seq (cog-link 'ConnectorSeq start con end)))
					(if (null? seq) '()
						(ListLink any-left seq)))))

		(define (get-right-wildcard WORD)
			(ListLink WORD any-right))

		(define (get-wild-wild)
			(ListLink any-left any-right))

		; Fetch (from the database) all sections
		(define (fetch-sections)
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
				((get-pair) get-pair)
				((make-pair) make-pair)
				((left-element) get-left-element)
				((right-element) get-right-element)
				((left-wildcard) get-left-wildcard)
				((right-wildcard) get-right-wildcard)
				((wild-wild) get-wild-wild)
				((fetch-pairs) fetch-sections)
				((provides) (lambda (symb) #f))
				((filters?) (lambda () #f))
				(else (error "Bad method call on cross-section:" message)))
			args)))
)

; ---------------------------------------------------------------------
