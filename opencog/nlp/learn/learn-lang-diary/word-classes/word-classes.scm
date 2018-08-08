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

; Print all of the classes that a word belongs to.
(define (prt-class-mebership WRD)
	(define membs (cog-incoming-by-type WRD 'MemberLink))
	(define classes (map gdr membs))
	(format #t "Word '~A' belongs to ~A classes:\n   "
		(cog-name WRD) (length classes))
	(for-each
		(lambda (cls) (format #t "<~A> " (cog-name cls)))
		classes)
	(newline))

; Print words that belong to more than one class
(define (prt-multi-members)
	(define nwrds 0)
	(define ducls '())
	(define (summer WRD)
		(define membs (cog-incoming-by-type WRD 'MemberLink))
		(if (< 1 (length membs))
			(begin
				(prt-class-mebership WRD)
				(set! nwrds (+ nwrds 1))
				(set! ducls (append ducls membs))))
	)
	(for-each summer (cog-get-atoms 'WordNode))
	(format #t "total words=~A total classes=~A unique classes=~A\n"
		nwrds (length ducls)
		(length (remove-duplicate-atoms (map gdr ducls))))
)

; -----------------------------------------------------------------
; -----------------------------------------------------------------
; -----------------------------------------------------------------
; Print a distribution of some statistic, vs. the class.
; First printed column: numerical ID for the class.
; Second printed column: The statistic.
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

(define (get-count atom)
	(if (cog-atom? atom) (cog-tv-count (cog-tv atom)) 0))

; Print disjunct support distribution, viz, number of distinct different
; disjuncts on each class
; Viz, this is the "support" or l_0 size.
; First column: numerical ID for the class.
; Second column: number of disjuncts in that class.
(define (prt-disjunct-support-distribution)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'Section)))
	(prt-distribution nmemb))

; Print disjunct-count distribution, viz, total number of observations
; of disjuncts on each class
; Viz, this is the "count" or l_1 size.
; First column: numerical ID for the class.
; Second column: sum of observations of disjuncts in that class.
(define (prt-disjunct-count-distribution)
	(define (nmemb CLS)
		(fold (lambda (SECT cnt) (+ cnt (get-count SECT))) 0
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
			(* (get-count SECT) (get-count SECT)))) 0
 			(cog-incoming-by-type CLS 'Section))))
	(prt-distribution nmemb))
;
; -----------------------------------------------------------------
;
; Print the distribution of disjuncts by disjunct-size.
; The disjunct-size is the number of connectors in a disjunct.
; Every section has exactly one disjunct in it.
; Loop over all sections on all word-classes, and count how many
; of these sections have the indicated size.
(define (prt-dj-size-distribution)
	(define (num-sections SIZ)
		(fold (lambda (CLS SUM)
				(+ SUM (length (get-sections-by-size CLS SIZ))))
			0
			(cog-get-atoms 'WordClassNode)))

	(define (prt-dist SIZ)
		(format #t "~A	~A\n" SIZ (num-sections SIZ)))

	(format #t "disjunct-size vs num-disjuncts\n")
	(list-tabulate 15 prt-dist)
)
;
; Print the distribution of disjuncts by disjunct-size.
; The disjunct-size is the number of connectors in a disjunct.
; Every section has exactly one disjunct in it.
; Loop over all sections on all word-classes, and count how many
; of these sections have the indicated size.
(define (prt-dj-weighted-size-distribution)
	(define (sum-section-weights SEC-LST)
		(fold (lambda (SEC SUM) (+ SUM (get-count SEC))) 0 SEC-LST))

	(define (weighted-num-sections SIZ)
		(fold (lambda (CLS SUM)
				(+ SUM (sum-section-weights (get-sections-by-size CLS SIZ))))
			0
			(cog-get-atoms 'WordClassNode)))

	(define (prt-dist SIZ)
		(format #t "~A	~A\n" SIZ (weighted-num-sections SIZ)))

	(format #t "disjunct-size vs weighted-num-disjuncts\n")
	(list-tabulate 15 prt-dist)
)
