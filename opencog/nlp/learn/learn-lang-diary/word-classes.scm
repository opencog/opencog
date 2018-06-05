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
	(format #t "Class <~A> has ~A members:\n"
		(cog-name CLS) (length membs))
	(for-each
		(lambda (memb)
			(format #t "      ~A\n" (cog-name (gar memb))))
		membs))

; Print all classes and members
(for-each prt-members-of-class (cog-get-atoms 'WordClassNode))

