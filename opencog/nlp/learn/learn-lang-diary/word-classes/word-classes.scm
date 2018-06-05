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

; Print size distribution
(define (prt-distribution)
	(define (nmemb CLS) (length (cog-incoming-by-type CLS 'MemberLink)))
	(define all-classes (cog-get-atoms 'WordClassNode))
	(define by-size
		(sort! all-classes
			(lambda (CLS-A CLS-B) (> (nmemb CLS-A) (nmemb CLS-B)))))
	(define cnt 1)
	(for-each
		(lambda (CLS)
			(format #t "~A	~A\n" cnt (nmemb CLS))
			(set! cnt (+ cnt 1)))
		by-size))
