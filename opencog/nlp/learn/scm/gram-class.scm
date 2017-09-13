;
; gram-class.scm
;
; Merge words into grammatical categories.
;
; ---------------------------------------------------------------------
; OVERVIEW
; --------
; When a pair of words are judged to be grammatically similar, they
; can be used to create a "grammatical class", containing both the
; words, and behaving as thier union/sum.  When a word is judged to
; belong to an existing grammatical-class, then some mechanism must
; be provided to add that word to the class.  This file implements
; the tools for creating and managing such classes.  It does not
; dictate how to judge when words belong to a class; this is done
; independently of the structure of the classes themselves.
;
; A grammatical class is represented as
;
;     MemberLink
;         WordNode "wordy"      ; the word itself
;         WordClassNode "noun"  ; the grammatical class of the word.
;
; Word classes have a designated grammatic behavior, using Sections,
; behaving just like the pseudo-connectors on single words. Thus, either
; a WordNode or a WordClassNode can appear in a Connector link, as
; shown below.
;
;     Section
;         WordClassNode "noun"
;         ConnectorSeq
;             Connector
;                WordClassNode "verb" ; or possibly a WordNode
;                LgConnDirNode "+"
;             Connector
;                ....
;
; There are several ways of handling the merger of words into classes.
; Consider first merging 
; The "linear" approach is to intersect the co
;
; ---------------------------------------------------------------------

(use-modules (opencog) (opencog matrix))

(define (transfer-count LLOBJ WA WB DJ NUM)
"
  transfer-count WA WB DJ NUM - subtract NUM DJ's from WB and add to WA.

  WA and WB shoud be WordNodes or WordClassNodes.
  DJ should be a disjunct i.e. a ConnectorSeq
  NUM shoud be a floating-point number.
  LLOBJ is used to access counts on pairs.  Pairs are SectionLinks,
     that is, are (word,disjunct) pairs wrapped in a SectionLink.

  If the pair (WB, DJ) exists, and has a count of at least NUM, then subtract NUM from the count of this section, and
  add it to the corresponding section on WA.  If there is no such
  section on WA, it is created.  If the count on WB is less than NUM,
  then the entire count is transfered to WA, and the section on WB
  is deleted.  NUM must be a positive number.
"
	(define sec-b (cog-link 'SectionLink WB DJ))
	(define vsec-b
		(if (null? sec-b) '() '()))

(define (func a b)
	(format #t "Its ~A and ~A\n" a b)
)

; (define ptu (add-tuple-math psa func))

#f
) 

(define (add-dynamic-stars LLOBJ)
"
  add-dynamic-stars LLOBJ - Extend LLOBJ with row and column access
  methods (aka wildcard methods), specifically, to get all non-zero
  elements in a given row or column.

  Similar to the (add-pair-stars LLOBJ) class, except that this
  attempts to work without having to load all pairs into RAM at the
  same time; instead, pairs are fetched from the database dynamically,
  on demand.  This is attempting to address the problem where not all
  of the matrix can fit into RAM at the same time.  If all of the
  matrix can fit, then (add-pair-stars LLOBJ) is more efficient.
  
"
	(let ((pair-type (LLOBJ 'pair-type))
		)

		(define (fetch-left left-item)
			(fetch-incoming-by-type left-item pair-type))

		;-------------------------------------------
		; Methods on this class.
		(lambda (message . args)
			(case message
				((feth-left)     (apply fetch-left args))
				(else            (apply LLOBJ (cons message args))))
		)))

(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-pair-stars pca))
			(pfa (add-pair-freq-api psa))
		)

		(load-atoms-of-type 'WordNode)

		; (define pta (make-thresh-pca pfa))
		; this can't work unless the whole matrix is already loaded...
		; (define all-words (get-all-cset-words))
		; brute force.
		(define all-words (cog-get-atoms 'WordNode))

	)
)
