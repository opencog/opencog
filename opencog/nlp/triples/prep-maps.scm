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
	(if 
		;; Make sure that the atom types are as expected, so that
		;; we don't create garbage.
		(and
			(eq? (cog-type verb) 'WordNode)
			(eq? (cog-type prep) 'DefinedLinguisticRelationshipNode)
		)
		(EvaluationLink
			(DefinedLinguisticRelationshipNode
				(string-append (cog-name verb) "_" (cog-name prep))
			)
			(ListLink
				(WordNode (cog-name verb))
				(DefinedLinguisticRelationshipNode (cog-name prep))
			)
		)
		'()
	)
)


(EvaluationLink (stv 1.0 1.0)
	(DefinedLinguisticRelationshipNode "capital_of")
	(ListLink
		(WordNode "capital")
		(DefinedLinguisticRelationshipNode "of")
	)
)

(EvaluationLink (stv 1.0 1.0)
	(DefinedLinguisticRelationshipNode "color_of")
	(ListLink
		(WordNode "color")
		(DefinedLinguisticRelationshipNode "of")
	)
)

(EvaluationLink (stv 1.0 1.0)
	(DefinedLinguisticRelationshipNode "make_from")
	(ListLink
		(WordNode "make")
		(DefinedLinguisticRelationshipNode "from")
	)
)

(EvaluationLink (stv 1.0 1.0)
	(DefinedLinguisticRelationshipNode "spin_from")
	(ListLink
		(WordNode "spin")
		(DefinedLinguisticRelationshipNode "from")
	)
)


.
exit
