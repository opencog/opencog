;
; prep-maps.scm
; Copyright (c) 2009 Linas Vepstas 
;
; Create maps linking word-preposition pairs to a single term
; These are needed to output word-prep-style triples.
;
; For example, given the input 
;    (make-prep-phrase (WordNode "capital")
;       (DefinedLinguisticRelationshipNode "of"))
;
; this will generate the following structure:
;
;    (EvaluationLink (stv 1.0 1.0)
;       (DefinedLinguisticRelationshipNode "capital_of")
;       (ListLink
;          (WordNode "capital")
;          (DefinedLinguisticRelationshipNode "of")
;       )
;    )


(define (make-prep-phrase word prep)
	(let* (
			(word-str (cog-name word))
			(prep-str (cog-name prep))
		)
		(if 
			;; Make sure that the atom types are as expected, and that
			;; the prep doesn't start with a leading underscore, so that
			;; we don't create garbage.
			(and
				(eq? (cog-type word) 'WordNode)
				(eq? (cog-type prep) 'DefinedLinguisticRelationshipNode)
				(not (eq? #\_ (car (string->list prep-str 0 1))))
			)
			(EvaluationLink (stv 1.0 1.0)
				(DefinedLinguisticRelationshipNode
					(string-append word-str "_" prep-str)
				)
				(ListLink
					(WordNode word-str)
					(DefinedLinguisticRelationshipNode prep-str)
				)
			)
			'()
		)
	)
)

; ---------------------------------------------------------
; Similar to above, except that we are promoting a ppolyword phrase
;
(define (make-polyword-phrase polyword)
	(let ((polyword-str (cog-name polyword)))
		(if (eq? (cog-type polyword) 'WordNode)
			(EvaluationLink (stv 1.0 1.0)
				(DefinedLinguisticRelationshipNode polyword-str)
				(ListLink
					(WordNode polyword-str)
					(DefinedLinguisticRelationshipNode polyword-str)
				)
			)
			'()
		)
	)
)
; ---------------------- END OF FILE ----------------------
