;
; compute-mi.scm
;
; Compute the mutual information of language word pairs.
;
; Copyright (c) 2013 Linas Vepstas
;
; ---------------------------------------------------------------------
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

; ---------------------------------------------------------------------
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

; ---------------------------------------------------------------------
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

; ---------------------------------------------------------------------
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

; ---------------------------------------------------------------------
; Compute the left and right word-pair wildcard counts.
; That is, compute the summations N(w,*) and N(*,w) where * denotes
; a wildcard, and ranges over all words observed in that slot.
; Store the resulting counts in a wild-card count structure, described
; below.
;
; To be precise, the summation is performed relative to the given
; LinkGrammar relationship node.  That is, the sumation only occurs
; over word pairs connected by the given link-grammar link.
; (Link grammar links are encoded with LinkGrammarRelationshipNode's).
;
; Thus, a word pair is currently represented as:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         WordNode "bird"
;
; To compute the left and right counts, we do an ad-hoc pattern
; match to the pattern below (ad-hoc because we don't bother with the
; pattern matcher here, the patten is too simple. In other cases, for
; structures more complex than word-pairs, we will need the matcher...)
; (Err, actually, we *do* use the pattern matcher, but the code would
; be faster and more efficient if we didn't. This should be fixed...)
;
; The match pattern for the right-counts is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         VariableNode of type WordNode   ; i.e. a wildcard here.
;
; while that for left-counts is:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         VariableNode of type WordNode  ; i.e. a wildcard on the left.
;         WordNode "bird"
;
; Sums are performed over all matching patterns (i.e. all values of the
; VariableNode).
;
; The resulting sums are stored in the CountTruthValues (on the
; EvaluationLink) of the following structures:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         AnyNode "left-word"
;         WordNode "bird"
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "ANY"
;      ListLink
;         WordNode "word"
;         AnyNode "right-word"
;
; This routine assumes that all relevant atoms are already in the atomspace.
; If they're not, incorrect counts will be obtained.
; Use the fetch-and-compute-pair-wildcard-counts below to fetch.

(define (compute-pair-wildcard-counts word lg_rel)

	; Define the bind links that we'll use with the pattern matcher.
	; left-bind has the wildcard on the left.
	(define left-bind-link
		(BindLink
			; Be careful to ask for WordNode, since there are also
			; eval links with AnyNode floating around ...
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
	; right-bind has the wildcard on the right.
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
			; lefties are those with the wildcard on the left side.
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

			; And now ... delete some of the crap we created.
			; Don't want to pollute the atomspace.
			(delete-hypergraph left-bind-link)
			(delete-hypergraph right-bind-link)
			; Note that cog-delete only goes one level deep, it does not
			; recurse; so the below only delete the ListLink at the top.
			(cog-delete left-list)
			(cog-delete right-list)

			; What the hell, return the two things
			(list left-star right-star)
		)
	)
)

; ---------------------------------------------------------------------
; fetch-and-compute-pair-wildcard-counts -- fetch wrapper around
; compute-pair-wildcard-counts
;
; Before compute-pair-wildcard-counts can do it's stuff, we have to
; make sure that all the relevant word-pairs are in the atomspace
; (viz. loaded from persistant store.)  After loading and computing,
; we delete these atoms, as they are too numerous to keep around.
;
(define (fetch-and-compute-pair-wildcard-counts word lg_rel)
	(let* (
			; inset is a list of ListLink's of word-pairs
			(inset (cog-incoming-set (fetch-incoming-set word)))
			; relset is a list of EvaluationLinks of word-pairs
			(relset (append-map
					(lambda (ll) (cog-incoming-set (fetch-incoming-set ll)))
					inset)
			)
			; The main, wrapped routine.
			(result (compute-pair-wildcard-counts word lg_rel))
		)
		(begin
			; Now, delete all the crap that we fetched.
			; OK, we want to do this: (for-each cog-delete relset)
			; but we can't, because this would also delete the wildcard
			; evaluation links. We want to keep those around. So ugly
			; filter.
			(for-each 
				(lambda (x) 
					; Returns true if either the left or right side is
					; the any-node
					(define (is-any? evl) 
						(define (zany atom) (eq? (cog-type atom) 'AnyNode))
						(if (zany (gadr evl)) #t (zany (gddr evl)))
					)

					(if (not (is-any? x)) (cog-delete x))
				)
				relset
			)

			(for-each cog-delete inset)
			result
		)
	)
)

; ---------------------------------------------------------------------
; Compute wildcard counts for all word-pairs, for the relation lg_rel.
; lg_rel is a link-grammar link type.
;
; This loops over all words, and computes the one-sided wild-card
; counts for these. Only the counts for the link-grammar link type
; lg_rel are computed.
;
; Caution: this can take hours or days to run.
;
(define (all-pair-wildcard-counts lg_rel)
	(begin
		; Make sure all words are in the atomspace
		(load-atoms-of-type 'WordNode)

		; For each word, fetch the word-pairs it occurs in, compute
		; the counts, and then delete the word-pairs. (saving teh counts).
		(for-each 
			(lambda (word) 
				(fetch-and-compute-pair-wildcard-counts word lg_rel)
			)
			(cog-get-atoms 'WordNode)
		)
	)
)

; ---------------------------------------------------------------------
; Compute wildcard counts for all word-pairs, for the relation lg_rel.
; lg_rel is a link-grammar link type.
;
; This loops over all words, and computes the one-sided wild-card
; counts for these. Only the counts for the link-grammar link type
; lg_rel are computed.
;
; The computation is done batch-style; all word-pairs are pulled into
; the atomspace first, and then the sums are taken. This can blow up
; RAM usage.
;
(define (batch-all-pair-wildcard-counts lg_rel)
	(begin
		; Make sure all words are in the atomspace
		(load-atoms-of-type 'WordNode)
		; Make sure all word-pairs are in the atomspace.
		(fetch-incoming-set lg_rel)
		; Compute the counts
		(for-each 
			(lambda (word) 
				(compute-pair-wildcard-counts word lg_rel)
			)
			(cog-get-atoms 'WordNode)
		)
	)
)

; ---------------------------------------------------------------------
; Compute the total number of observed word-pairs for the link-grammar
; link (relation) lg_rel.
;
; The resulting total count is stored as the count on the EvaluationLink
; for the structure
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         AnyNode "left-word"
;         AnyNode "right-word"
;
; where lg_rel is (LinkGrammarRelationshipNode "Blah")
;
; The computation is performed batch-style. This function assumes that
; all word-pairs are already loaded in the atom table; it will get
; incorrect counts if this is not the case.
;
(define (batch-all-pair-count lg_rel)
	(define l-cnt 0)
	(define r-cnt 0)
	(begin
		; Now, loop over all words, totalling up the counts.
		(for-each 
			(lambda (word) 
				(set! l-cnt
					(+ l-cnt (get_left_wildcard_count word lg_rel))
				)
				(set! r-cnt
					(+ r-cnt (get_right_wildcard_count word lg_rel))
				)
			)
			(cog-get-atoms 'WordNode)
		)

		; The left and right counts should be equal
		(if (not (eq? l-cnt r-cnt))
			(display "Error: word-pair-counts unequal: ")
			(display l-cnt)
			(display " ")
			(display r-cnt)
			(display "\n")
		)

		; Create and save the grand-total count.
		(store-atom 
			(EvaluationLink (cog-new-ctv 0 0 r-cnt)
				lg_rel
				(ListLink
					(AnyNode "left-word")
					(AnyNode "right-word")
				)
			)
		)
	)
)

; ---------------------------------------------------------------------
; Compute the log-liklihood for all wild-card wordpairs, for lg_rel.
; lg_rel is a link-gramar link (LinkGrammarRelationshipNode)
;
; This computes the logliklihood for all wild-card pairs of the form:
;
;   EvaluationLink
;      LinkGrammarRelationshipNode "Blah"
;      ListLink
;         WordNode "some-word"
;         AnyNode "right-word"
;
; and also for the flipped version (exchange WordNode and AnyNode)
; Here, lg_rel is (LinkGrammarRelationshipNode "Blah")
;
; The log likelihood is stored in the 'confidence' slot on the eval
; link truth value (where logli's are always stored).
;
; The computation is performed "batch style", it assumes that all
; words in the atomspace; missing words will cause the corresponding
; pairs to be missed.

(define (batch-all-pair-wildcard-logli lg_rel)

	; Get the word-pair grand-total
	(define pair-total
		; Return the count for all pairs.
		(tv-count (cog-tv 
			(EvaluationLink lg_rel
				(ListLink (AnyNode "left-word") (AnyNode "right-word"))
			)
		))
	)

	; For each wild-card pair associated with the word,
	; obtain the log likelihood.
	(for-each 
		(lambda (word)
			; log-likelihood for the left wildcard
			(store-atom 
 				(compute-atom-logli
					(EvaluationLink lg_rel (ListLink (AnyNode "left-word") word))
					pair-total
				)
			)
			; log-likelihood for the right wildcard
			(store-atom 
 				(compute-atom-logli
					(EvaluationLink lg_rel (ListLink word (AnyNode "right-word")))
					pair-total
				)
			)
		)
		(cog-get-atoms 'WordNode)
	)
)

; ---------------------------------------------------------------------

(define (batch-all-pair-mi lg_rel)
		(begin
			; First, get the left and right wildcard counts.
			(batch-all-pair-wildcard-counts lg_rel)
(display "duuuuude done with pair wildcards\n")

			; Now, get the grand-total
			(batch-all-pair-count lg_rel)
(display "duuuuude done with grand-total\n")

)

(define (do-em-all)
	(batch-all-pair-wildcard-logli (LinkGrammarRelationshipNode "ANY"))
)

; ---------------------------------------------------------------------
; Get the left wild-card count for word and lg_rel.
; (that is, the wildcard is on the left side)

(define (get_left_wildcard_count word lg_rel)
	(define evl
		(EvaluationLink lg_rel (ListLink (AnyNode "left-word") word))
	)
	(tv-count (cog-tv evl))
)

; ---------------------------------------------------------------------
; Get the right wild-card count for word and lg_rel.
; (that is, the wildcard is on the right side)

(define (get_right_wildcard_count word lg_rel)
	(define evl
		(EvaluationLink lg_rel (ListLink word (AnyNode "right-word")))
	)
	(tv-count (cog-tv evl))
)

; ---------------------------------------------------------------------
; misc hand debug stuff
;
; (define x (WordNode "famille"))
; (define y (LinkGrammarRelationshipNode "ANY"))
; (fetch-and-compute-pair-wildcard-counts x y)
;
; (load-atoms-of-type 'WordNode)
; (define wc (cog-count-atoms 'WordNode))
; (length (cog-get-atoms 'WordNode))
; (define wc (get-total-atom-count (cog-get-atoms 'WordNode)))
;
; (compute-word-prob x wc)
;
; select count(*) from  atoms where type = 77;
; 16785 in fr
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
;
; (define wtfl  (EvaluationLink  (LinkGrammarRelationshipNode "ANY")
;   (ListLink (AnyNode "left-word") (WordNode "famille"))))
;
; (define wtfr  (EvaluationLink  (LinkGrammarRelationshipNode "ANY")
;     (ListLink (WordNode "famille") (AnyNode "right-word"))))
;
;
; anynode is type 105
;  select * from atoms where type=105;
; uuid is 43464152
;         43464155
; select count(*) from atoms where outgoing @> ARRAY[cast(43464152 as bigint)];
; returns the number of word-pairs which we've wild-carded.

