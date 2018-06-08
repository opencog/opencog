;
; grammatical class explorer
; Display stats and stuff, about the various word classes.
;
; -------------------------------------------------------
(use-modules (srfi srfi-1))
;
; Load all grammatical classes from storage
(define (fetch-all-gram-classes)
	(load-atoms-of-type 'WordClassNode)
	(for-each
		(lambda (CLS) (fetch-incoming-by-type CLS 'MemberLink))
		(cog-get-atoms 'WordClassNode)))

; How many grammatical classes are there?
(cog-count-atoms 'WordClassNode)

; Show them all
(cog-get-atoms 'WordClassNode)

; How many words have been classified?
(define (num-classified-words)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
	(fold (lambda (CLS cnt) (+ cnt (nmemb CLS))) 0
		(cog-get-atoms 'WordClassNode)))

; Print the members of one class.
; Print most frequent words first.
; (Roughly. The get-count is not really acccurate; it includes totals
; from pair counting, not MST counting.  And the MST counts get trashed
; during classification, so we don't have any good counts left...)
(define (prt-members-of-class CLS)
	(define membs (cog-incoming-by-type CLS 'MemberLink))
	(define words (map gar membs))
	(define words-by-freq
		(sort! words
			(lambda (WRD-A WRD-B) (> (get-count WRD-A) (get-count WRD-B)))))
	(format #t "Class <~A> has ~A members:\n   "
		(cog-name CLS) (length words-by-freq))
	(for-each
		(lambda (wrd)
			(format #t "~A " (cog-name wrd)))
		words-by-freq)
	(newline))

; Print all classes and members
(define (prt-all-classes)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
	(define all-classes (cog-get-atoms 'WordClassNode))
	(define by-size
		(sort! all-classes
			(lambda (CLS-A CLS-B) (> (nmemb CLS-A) (nmemb CLS-B)))))
	(format #t "There are ~A words placed into ~A classes\n"
		(num-classified-words) (cog-count-atoms 'WordClassNode))
	(for-each prt-members-of-class by-size))

; Print distribution, viz, Some statistic, vs. the class.
; First column: numerical ID for the class.
; Second column: The statistic.
; The FUNC  should be a function taking the class, returning a number.
(define (prt-distribution FUNC)
	(define all-classes (cog-get-atoms 'WordClassNode))
	(define by-size
		(sort! all-classes
			(lambda (CLS-A CLS-B) (> (FUNC CLS-A) (FUNC CLS-B)))))
	(define cnt 1)
	(for-each
		(lambda (CLS)
			(format #t "~A	~A\n" cnt (FUNC CLS))
			(set! cnt (+ cnt 1)))
		by-size))

; Print size distribution, viz, number of member-words.
; First column: numerical ID for the class.
; Second column: number of words in that class.
(define (prt-word-distribution)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
	(prt-distribution nmemb))

; Print disjunct support distribution, viz, number of disjuncts on
; each class
; Viz, this is the "support" or l_0 size.
; First column: numerical ID for the class.
; Second column: number of disjuncts in that class.
(define (prt-disjunct-distribution)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'Section)))
	(prt-distribution nmemb))

; Print disjunct-count distribution, viz, total number of observations
; of disjuncts on each class
; Viz, this is the "count" or l_1 size.
; First column: numerical ID for the class.
; Second column: sum of observations of disjuncts in that class.
(define (prt-disjunct-count-distribution)
	(define (nmemb CLS)
		(fold (lambda (SECT cnt) (+ cnt (cog-tv-count(cog-tv SECT)))) 0
 			(cog-incoming-by-type CLS 'Section)))
	(prt-distribution nmemb))

; Print disjunct-length distribution, viz, mean-square number of
; observations of disjuncts on each class
; Viz, this is the "length" or l_2 size.
; First column: numerical ID for the class.
; Second column: mean-square number of observations of disjuncts in that class.
(define (prt-disjunct-length-distribution)
	(define (nmemb CLS) (sqrt
		(fold (lambda (SECT cnt) (+ cnt 
			(* (cog-tv-count(cog-tv SECT)) (cog-tv-count(cog-tv SECT))))) 0
 			(cog-incoming-by-type CLS 'Section))))
	(prt-distribution nmemb))
; 
