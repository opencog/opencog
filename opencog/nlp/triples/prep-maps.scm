scm
;
; prep-maps.scm
;
; Create maps linking verb-preposition pairs to a single term
; These are needed to output verb-prep-style triples.
;
; For example, given the input 
;    (make-verb-prep (WordNode "capital")
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


(define (make-verb-prep verb prep)
	(let* (
			(verb-str (cog-name verb))
			(prep-str (cog-name prep))
		)
		(if 
			;; Make sure that the atom types are as expected, and that
			;; the prep doesn't start with a leading underscore, so that
			;; we don't create garbage.
			(and
				(eq? (cog-type verb) 'WordNode)
				(eq? (cog-type prep) 'DefinedLinguisticRelationshipNode)
				(not (eq? #\_ (car (string->list prep-str 0 1))))
			)
			(EvaluationLink (stv 1.0 1.0)
				(DefinedLinguisticRelationshipNode
					(string-append verb-str "_" prep-str)
				)
				(ListLink
					(WordNode (cog-name verb))
					(DefinedLinguisticRelationshipNode (cog-name prep))
				)
			)
			'()
		)
	)
)

.
exit
