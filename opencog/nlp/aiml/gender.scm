;
; Person and gender conversion.
;

; AIML-tag person -- Convert 1st to third person, and back.
(define-public (do-aiml-person TEXT)
	(define (cvt WORD)
		(define ws (cog-name WORD))
		(cond
			((equal? ws "we") "they")
			((equal? ws "me") "they")
			((equal? ws "they") "I")
			((equal? ws "us") "them")
			((equal? ws "them") "us")
			(else ws)
		)
	)
	(define (wcvt str) (Word (cvt str)))
	(ListLink (map wcvt (cog-outgoing-set TEXT)))
)

; AIML-tag person2 -- Convert 1st to second person, and back.
(define-public (do-aiml-person2 TEXT)
	TEXT
)

; AIML-tag gender -- Convert male to female and back.
(define-public (do-aiml-gender TEXT)
	TEXT
)

; ==============================================================
;; mute the thing
*unspecified*
