; ===========================================================================
; Psi Emotion Representations

(define psi-emotion-node (Concept (string-append psi-prefix-str "emotion")))

(define (psi-create-emotion emotion)
	(define emotion-concept (Concept (string-append psi-prefix-str emotion)))
	(Inheritance emotion-concept psi-emotion-node)
	; initialize value ?
	(psi-set-value! emotion-concept 0)
	;(format #t "new emotion: ~a\n" emotion-concept)
	emotion-concept)

(define (psi-get-emotions)
"
	Returns a list of all psi emotions.
"
	(filter
		(lambda (x) (not (equal? x psi-emotion-node)))
		(cog-chase-link 'InheritanceLink 'ConceptNode psi-emotion-node))
)
