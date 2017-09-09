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

(define (transfer-count WA WB DJ NUM)
"
  transfer-count WA WB DJ NUM - subtract NUM DJ's from WB and add to WA.

  If WB is in a section with the connector-seq DJ, and has a count of
  at least NUM, then subtract NUM from the count of this section, and
  add it to the corresponding section on WA.  If tehre is no such
  section on WA, it is created.  If the count on WB is less than NUM,
  then the entire count is transfered to WA, and the section on WB
  is deleted.  NUM must be a positie number.
"
	(define sec-b (cog-link 'SectionLink WB DJ))
	(define vsec-b
		(if (null? sec-b) '()
			(cog-value sec-b 
) 

(define (do-it)
	(let ((pca (make-pseudo-cset-api))
			(psa (add-pair-stars pca))
			(pfa (add-pair-freq-api psa))
		)

	(define pta (make-thresh-pca pfa))
	(define all-words (get-all-cset-words))
)
