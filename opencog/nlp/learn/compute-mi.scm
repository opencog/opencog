;
; compute-mi.scm
;
; Compute the mutual information of language word pairs.
;
; Copyright (c) 2013 Linas Vepstas
;
; ----------------------------------------------------
; Count the total number of times that all words have been observed.
;
; Do this by obtaining all WordNodes, and adding up the 'count' values
; on their CountTruthValue.

(define (get-total-word-count)
	(let ((cnt 0))
		(define (inc atom)
			(set! cnt 
				(+ cnt 
					(assoc-ref (cog-tv->alist (cog-tv atom)) 'count)
				)
			)
			#f
		)
		(cog-map-type inc 'WordNode)
		cnt
	)
)

; ----------------------------------------------------
; Compute frequency of having observed a given word.
;
; The frequency will be stored in the atom's TV 'confidence' location.
; The frequency is computed by simply taking the atom's count value,
; and dividing by the total.

(define (compute-word-freq atom total)
	(let* (
			(atv (cog-tv->alist (cog-tv atom)))
			(meen (assoc-ref atv 'mean))
			(cnt (assoc-ref atv 'count))
			(ntv (cog-new-ctv meen (/ cnt total) cnt))
		)
		(cog-set-tv! atom ntv)
	)
)

; ----------------------------------------------------
; Compute the occurance frequencies for all words.
;
; Load all word-nodes into the atomspace, first, so that an accurate
; count of wrd-occurances can be obtained.  The frequency for a given
; word node is stored in the 'confidence' slot of the CountTruthValue.

(define (compute-all-word-freqs)
	(begin
		; Make sure that all word-nodes are in the atom table.
		(load-atoms-of-type 'WordNode)
		(let ((total (get-total-word-count)))
			(cog-map-type
				(lambda (atom) (compute-word-freq atom total) #f)
				'WordNode
			)
		)
	)
)

; ----------------------------------------------------
; misc hand debug stuff

(define x (WordNode "famille"))

(load-atoms-of-type 'WordNode)
(define wc (cog-count-atoms 'WordNode))
(define wc (get-total-word-count))

(compute-word-prob x wc)

select count(uuid) from  atoms where type = 77;
12199 in fr
19781 in lt

