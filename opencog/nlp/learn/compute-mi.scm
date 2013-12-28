;
; compute-mi.scm
;
; Compute the mutual information of language word pairs.
;
; Copyright (c) 2013 Linas Vepstas
;
; ----------------------------------------------------
; Count the total number of times that the atoms in the atom-list have
; been observed.  The observation-count for a single atom is stored in
; the 'count' value of its CountTruthValue. This routine just fetches
; those, and addes them up.
;
; The returned value is the total count.

(define (get-total-atom-count atom-list)
	(let ((cnt 0))
		(define (inc atom) (set! cnt (+ cnt (tv-count (cog-tv atom)))))
		(for-each inc atom-list)
		cnt
	)
)

; ----------------------------------------------------
; Compute log liklihood of having observed a given atom.
;
; The liklihood will be stored in the atom's TV 'confidence' location.
; The log liklihood is -log_2(frequency), with the frequency computed
; by simply taking the atom's count value, and dividing by the total.
;
; This returns the atom that was provided, but now with the logli set.

(define (compute-atom-logli atom total)
	(let* (
			(atv (cog-tv->alist (cog-tv atom)))
			(meen (assoc-ref atv 'mean))
			(cnt (assoc-ref atv 'count))
			; 1.4426950408889634 is 1/0.6931471805599453 is 1/log 2
			(ln2 (* -1.4426950408889634 (log (/ cnt total))))
			(ntv (cog-new-ctv meen ln2 cnt))
		)
		(cog-set-tv! atom ntv)
	)
)

; ----------------------------------------------------
; Compute the occurance logliklihoods for a list of atoms.
;
; This sums up the occurance-count over the entire list of atoms,
; and uses that as the normalization for the probability frequency
; for the individual atoms in the list. It then computes the log_2
; likelihood for each atom in the list, based on the total.
;
; As usual, the raw counts are obtained from the 'count' slot on a
; CountTruthValue, and the logli is stored in the 'confidence' slot.
;
; This returns the atom-list, but now with the logli's set.

(define (compute-all-logli atom-list)
	(let ((total (get-total-atom-count atom-list)))
		(map
			(lambda (atom) (compute-atom-logli atom total))
			atom-list
		)
	)
)

; ----------------------------------------------------
; Compute the occurance logliklihoods for all words.
;
; Load all word-nodes into the atomspace, first, so that an accurate
; count of word-occurances can be obtained.  The loglikli for a given
; word node is stored in the 'confidence' slot of the CountTruthValue.
;
; This returns the list of all word-nodes, with the logli's set.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(load-atoms-of-type 'WordNode)
		(compute-all-logli (cog-get-atoms 'WordNode))
	)
)

; ----------------------------------------------------
; Compute the left and right word-pair log liklihoods.
;
; The computation is performed relative to the LinkGrammar relationship
; node. (Currently "ANY", but will change as learning progresses.)
; Thus, a word pair is currently represented as:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         WordNode "bird"
;
; To compute the left and right frequencies, we do an ad-hoc pattern
; match to the pattern below (ad-hoc because we don't bother with the
; pattern matccher here, the patten is too simple. In other cases, for
; structures more complex than word-pairs, we will need the matcher...)
; The match pattern for the left-frequencies is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         VariableNode of type WordNode
;
; while that for right-frequencies is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         VariableNode of type WordNode
;         WordNode "bird"
;
; Sums are performed over all matching patterns (i.e. all values of the
; VariableNode) Normalization is with respect to the count on the fixed
; WordNode.
;
; The resulting sums are stored in the CountTruthValues (on the
; EvaluationLink) of the following structures:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         AnyNode "right-word"
;         WordNode "bird"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         AnyNode "left-word"
;
; Before the above is accomplished, we have to make sure that all links
; of the above type; are in the atomtable, viz, have been loaded from
; persistant store. After doing the above, we delete these atoms, as they
; are too numerous to keep around.

(define (compute-left-pair-logli word lg_rel)
	(define left-bind-link
		(BindLink
			(TypedVariableLink
				(VariableNode "$left-word")
				(VariableTypeNode "WordNode")
			)
			(ImplicationLink
				(EvaluationLink
					lg_rel
					(ListLink
						(VariableNode "$left-word")
						word
					)
				)
				(EvaluationLink
					lg_rel
					(ListLink
						(VariableNode "$left-word")
						word
					)
				)
			)
		)
	)
	(define right-bind-link
		(BindLink
			(TypedVariableLink
				(VariableNode "$right-word")
				(VariableTypeNode "WordNode")
			)
			(ImplicationLink
				(EvaluationLink
					lg_rel
					(ListLink
						word
						(VariableNode "$right-word")
					)
				)
				(EvaluationLink
					lg_rel
					(ListLink
						word
						(VariableNode "$right-word")
					)
				)
			)
		)
	)
	(let* (
			; inset is a list of ListLink's of word-pairs
			(inset (cog-incoming-set (fetch-incoming-set word)))
			; relset is a list of EvaluationLinks of word-pairs
			(relset (append-map
					(lambda (ll) (cog-incoming-set (fetch-incoming-set ll)))
					inset)
			)
			; lefties are those with the varying left side only.
			; XXX It would be more efficient to not use the pattern
			; matcher here, but to instead filter the given relset.
			; But I'm lazy, for just right now.
			(left-list (cog-bind left-bind-link))
			(right-list (cog-bind right-bind-link))
			(lefties (cog-outgoing-set left-list))
			(righties (cog-outgoing-set right-list))

			; the total occurance counts
			(left-total (get-total-atom-count lefties))
			(right-total (get-total-atom-count righties))

		)
		(begin
			; Create the two evaluation links to hold the counts.
			(define left-star
				(EvaluationLink (cog-new-ctv 0 0 left-total)
					lg_rel
					(ListLink
						(AnyNode "left-word")
						word
					)
				)
			)
			(define right-star
				(EvaluationLink (cog-new-ctv 0 0 right-total)
					lg_rel
					(ListLink
						word
						(AnyNode "right-word")
					)
				)
			)

			; Save these hard-won counts to the database.
			(store-atom left-star)
			(store-atom right-star)

			; And now ... delete all the atoms we fetched, so we don't
			; glom up the atomspace.
			(delete-hypergraph left-bind-link)
			(delete-hypergraph right-bind-link)
			(cog-delete left-list)
			(cog-delete right-list)
			(for-each cog-delete relset)
			(for-each cog-delete inset)

			(list left-star right-star)
		)
	)
)

; ----------------------------------------------------
; misc hand debug stuff
;
; (define x (WordNode "famille"))
; (define y (LinkGrammarRelationshipNode "ANY"))
; (compute-left-pair-logli  x y)
;
; (load-atoms-of-type 'WordNode)
; (define wc (cog-count-atoms 'WordNode))
; (define wc (get-total-atom-count (cog-get-atoms 'WordNode)))
;
;
; (compute-word-prob x wc)
;
; select count(uuid) from  atoms where type = 77;
; 12199 in fr
; 19781 in lt
;
; select * from atoms where name='famille';
; uuid is 2908473
; select * from atoms where outgoing @> ARRAY[2908473];
; select * from atoms where outgoing @> ARRAY[cast(2908473 as bigint)];
;
; 43464154
; duuude left-star handle is 
; 43464157duuude good by


