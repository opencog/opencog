;
; grammatical class explorer
; Display stats and stuff, about the various word classes.
;
; -------------------------------------------------------
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
(define (prt-members-of-class CLS)
	(define membs (cog-incoming-by-type CLS 'MemberLink))
	(format #t "Class <~A> has ~A members:\n   "
		(cog-name CLS) (length membs))
	(for-each
		(lambda (memb)
			(format #t "~A " (cog-name (gar memb))))
		membs)
	(newline))

; Print all classes and members
(define (prt-all-classes)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
	(define all-classes (cog-get-atoms 'WordClassNode))
	(define by-size
		(sort! all-classes
			(lambda (CLS-A CLS-B) (> (nmemb CLS-A) (nmemb CLS-B)))))
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

; Print disjunct distribution, viz, number of disjuncts on each class
; First column: numerical ID for the class.
; Second column: number of disjuncts in that class.
(define (prt-disjunct-distribution)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'Section)))
	(prt-distribution nmemb))
; 
