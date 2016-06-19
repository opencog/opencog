;
; Person and gender conversion.
;

; AIML-tag person -- Convert 1st to third person, and back.
(define-public (do-aiml-person TEXT)
	(define (cvt WORD)
		(define ws (cog-name WORD))
		(cond
			((equal? ws "I") "he or she")
			((equal? ws "he") "I")
			((equal? ws "she") "I")
			((equal? ws "we") "they")
			((equal? ws "me") "they")
			((equal? ws "they") "I")
			((equal? ws "us") "them")
			((equal? ws "them") "us")
			((equal? ws "mine") "thier")
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
