;
; Person and gender conversion.
;

; AIML-tag person -- Convert 1st to third person, and back.
(define-public (do-aiml-person TEXT)
	(define (cvt WORD)
		(define ws (cog-name WORD))
		(cond
			((string-ci=? ws "I") "he")
			((equal? ws "me") "him")
			((equal? ws "my") "his")
			((equal? ws "mine") "his")
			((equal? ws "myself") "himself")
			((equal? ws "am") "is")
			((equal? ws "he") "I")
			((equal? ws "him") "me")
			((equal? ws "his") "my")
			((equal? ws "himself") "myself")
			((equal? ws "is") "am")
			((equal? ws "she") "I")
			((equal? ws "her") "me")
			((equal? ws "hers") "mine")
			((equal? ws "herself") "myself")
			((equal? ws "we") "they")
			((equal? ws "us") "them")
			((equal? ws "they") "I")
			((equal? ws "them") "us")
			(else ws)
		)
	)
	(define (wcvt str) (Word (cvt str)))
	(ListLink (map wcvt (cog-outgoing-set TEXT)))
)

; AIML-tag person2 -- Convert 1st to second person, and back.
(define-public (do-aiml-person2 TEXT)
	(define (cvt WORD)
		(define ws (cog-name WORD))
		(cond
			((equal? ws "you") "me")
			((equal? ws "me") "you")
			((equal? ws "your") "my")
			((equal? ws "my") "your")
			((equal? ws "yours") "mine")
			((equal? ws "mine") "yours")
			((equal? ws "yourself") "myself")
			((equal? ws "myself") "yourself")
			(else ws)
		)
	)
	(define (wcvt str) (Word (cvt str)))
	(ListLink (map wcvt (cog-outgoing-set TEXT)))
)

; AIML-tag gender -- Convert male to female and back.
(define-public (do-aiml-gender TEXT)
	(define (cvt WORD)
		(define ws (cog-name WORD))
		(cond
			((equal? ws "his") "her")
			((equal? ws "her") "his")
			((equal? ws "him") "her")
			(else ws)
		)
	)
	(define (wcvt str) (Word (cvt str)))
	(ListLink (map wcvt (cog-outgoing-set TEXT)))
)

; ==============================================================
;; mute the thing
*unspecified*
