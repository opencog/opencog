scm
;
; prep-maps.scm
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
			(word-str (cog-name verb))
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
					(WordNode (cog-name word))
					(DefinedLinguisticRelationshipNode (cog-name prep))
				)
			)
			'()
		)
	)
)

.
exit
