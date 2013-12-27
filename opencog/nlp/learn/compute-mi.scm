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
; Compute the occurance logliklihoods for all words.
;
; Load all word-nodes into the atomspace, first, so that an accurate
; count of word-occurances can be obtained.  The loglikli for a given
; word node is stored in the 'confidence' slot of the CountTruthValue.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(load-atoms-of-type 'WordNode)
		(let ((total (get-total-atom-count (cog-get-atoms 'WordNode))))
			(cog-map-type
				(lambda (atom) (compute-atom-logli atom total) #f)
				'WordNode
			)
		)
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
; Before the above is accomplished, we have to make sure that all links
; of the above type; are in the atomtable, viz, have been loaded from
; persistant store. After doing the above, we delete these atoms, as they
; are too numerous to keep around.

(define (compute-left-pair-logli word lg_rel)
	(define bind-link 
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
	(let* (
			; inset is a list of ListLink's of word-pairs
			(inset (cog-incoming-set (fetch-incoming-set word)))
			; relset is a list of EvaluationLinks of word-pairs
			(relset (append-map 
					(lambda (ll) (cog-incoming-set (fetch-incoming-set ll)))
					inset)
			)
			; lefties are those with the varyig left side only.
			(lefties (cog-bind bind-link))
		)
	)
)

; ----------------------------------------------------
; misc hand debug stuff

(define x (WordNode "famille"))

(load-atoms-of-type 'WordNode)
(define wc (cog-count-atoms 'WordNode))
(define wc (get-total-atom-count (cog-get-atoms 'WordNode)))


(compute-word-prob x wc)

select count(uuid) from  atoms where type = 77;
12199 in fr
19781 in lt

select * from atoms where name='famille';
uuid is 2908473
select * from atoms where outgoing @> ARRAY[2908473];
select * from atoms where outgoing @> ARRAY[cast(2908473 as bigint)];

