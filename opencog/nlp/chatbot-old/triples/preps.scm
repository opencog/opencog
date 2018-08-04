;
; preps.scm
; Copyright (c) 2009 Linas Vepstas
;
; List of English-language prepositions.
; These are needed to allow the various triple-rules to be processed.
;
; Note that the list is quoted -- 
;
(define prep-master-list (list
'aboard
'about
'above
'across
'after
'against
'along
'alongside
'amid
'amidst
'among
'amongst
'around
'as
'aside
'at
'athwart
'atop
'barring
'before
'behind
'below
'beneath
'beside
'besides
'between
'beyond
'but
'by
'circa
'concerning
'despite
'down
'during
'except
'failing
'following
'for
'from
'in
'inside
'into
'like
'minus
'near
'next
'notwithstanding
'of
'off
'on
'onto
'opposite
'out
'outside
'over
'pace
'past
'per
'plus
'regarding
'round
'save
'since
'than
'through
'throughout
'till
'times
'to
'toward
'towards
'under
'underneath
'unlike
'until
'up
'upon
'versus
'via
'with
'within
'without
'worth
;
; two-word preps
;
'according_to
'ahead_of
'aside_from
'because_of
'close_to
'due_to
'except_for
'far_from
'in_to
'inside_of
'instead_of
'near_to
'next_to
'on_to
'out_from
'out_of
'outside_of
'owing_to
'prior_to
'pursuant_to
'regardless_of
'subsequent_to
'that_of
;
; three-word preps
;
'as_far_as
'as_well_as
'by_means_of
'in_accordance_with
'in_addition_to
'in_case_of
'in_front_of
'in_lieu_of
'in_place_of
'in_spite_of
'on_account_of
'on_behalf_of
'on_top_of
'with_regard_to
;
; postpositions
;
; five years ago
'ago
; this apart, ...
'apart
; five light years away
'away
; five years hence
'hence
))

; ----------------------------------------------------------------
; Accept as input a list of prepositions, and create a simple
; ListLink for each prep.  The ListLink is of the following pecurliar
; form:
;
; (ListLink
;    (DefinedLinguisticRelationshipNode "regardless_of")
;    (WordNode "regardless_of")
; )
;
; This oddball form is needed by the triples code to match up common
; WordNodes appearing in sentences to DefinedLinguisticRelationshipNodes
; appearing in triples.

(define (prep-make-relations prep-lst)

	(define (make-relation prep)
		(define sprep (symbol->string prep))
		(ListLink (stv 1 1)
			(DefinedLinguisticRelationshipNode sprep)
			(WordNode sprep)
		)
	)

	(for-each make-relation prep-lst)
)

; Now actually make the relations
(define prep-master-relations (prep-make-relations prep-master-list))

; --------------------------- END OF FILE --------------------------
