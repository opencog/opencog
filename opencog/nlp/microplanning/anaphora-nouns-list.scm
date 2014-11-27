(use-modules (oop goops))
(load-scm-from-file "../opencog/nlp/microplanning/anaphora-noun-item.scm")

(define-class <nouns-list> ()
	(lst #:init-value '())
)

(define-method (get-length (nl <nouns-list>))
	(length (slot-ref nl 'lst))
)

(define-method (add-noun-item (nl <nouns-list>) (ni <noun-item>))
	(slot-set! nl 'lst (append (slot-ref nl 'lst) (list ni)))
)

(define-method (get-noun-item (nl <nouns-list>) (index <integer>))
	(list-ref (slot-ref nl 'lst) index)
)

(define-method (get-sublist (nl <nouns-list>) (start-index <integer>) (end-index <integer>))
	(define new-list (make <nouns-list>))
	(slot-set! new-list 'lst (sublist (slot-ref nl 'lst) start-index end-index))
	new-list
)

(define-method (get-chunk-sublist (nl <nouns-list>) (chunk-index <integer>))
	; find subset of nouns-list that share the same chunk number
	(define start-index (list-index (lambda (n) (= chunk-index (get-chunk-index n))) (slot-ref nl 'lst)))
	(define end-index (list-index (lambda (n) (< chunk-index (get-chunk-index n))) (slot-ref nl 'lst)))

	; makes it so that we will return an empty subset if a chunk does not need to be changed
	; (eg. if a chunk somehow has no noun)
	(if (not start-index)
		(set! start-index (get-length nl))
	)
	; if we have reached the end of the set
	(if (not end-index)
		(set! end-index (get-length nl))
	)

	(get-sublist nl start-index end-index)
)

(define-method (get-chunk-link-sublist (nl <nouns-list>) (chunk-index <integer>) (link-index <integer>))
	(define chunk-sublist (get-chunk-sublist nl chunk-index))
	
	; find subset of chunk-sublist that share the same link number
	(define start-index (list-index (lambda (n) (= link-index (get-link-index n))) (slot-ref chunk-sublist 'lst)))
	(define end-index (list-index (lambda (n) (< link-index (get-link-index n))) (slot-ref chunk-sublist 'lst)))

	; makes it so that we will return an empty subset if a link does not need to be changed
	; (eg. if a link does not contain a noun)
	(if (not start-index)
		(set! start-index (get-length chunk-sublist))
	)

	; if we have reached the end of the subset
	(if (not end-index)
		(set! end-index (get-length chunk-sublist))
	)
	
	(get-sublist chunk-sublist start-index end-index)
)

(define-method (find-by-proc (nl <nouns-list>) (proc <procedure>))
	(find proc (slot-ref nl 'lst))
)

(define-method (is-modified? (nl <nouns-list>) (ni <noun-item>))
	; TODO add check where within the same chunk one noun appear multiple times
	;      and only one of them is modified
	;      eg.  "Bob collects the tiny seeds and plants seeds."
	; XXX  might need to call SuReal to do the above check
	(any (lambda (n) 
		(and (= (get-chunk-index n) (get-chunk-index ni))		; in the same chunk?
		     (equal? (get-noun-node n) (get-noun-node ni))		; with the same node?
		     (equal? 'InheritanceLink (cog-type (get-orig-link n)))	; in InheritanceLink modifying the noun (amod-rule)
		)
	     )
	     (slot-ref nl 'lst)
	)
)

; check the neighboring subject/object/indirect-object and see if the pronoun will be ambiguious
(define-method (is-ambiguous? (nl <nouns-list>) (ni <noun-item>))
	(define index (list-index equal? (slot-ref nl 'lst) (circular-list ni)))
	(define min-index (max 0 (- index 3))) ; inclusive
	(define max-index (min (get-length nl) (+ index 3))) ; exclusive

	; check how many pronouns in subset equals ni's (except itself)
	(> (count
		(lambda (n) (and (not (equal? 'InheritanceLink (cog-type (get-orig-link n)))) ; exclude modifying link from amod-rule
				 (not (equal? (get-noun-node n) (get-noun-node ni)))
				 (string=? (get-base-pronominal n) (get-base-pronominal ni))))
		(sublist (slot-ref nl 'lst) min-index max-index)
	   )
	   0
	)

	; TODO sometimes it is OK depends on the main subject (current and previous sentence)
	; (eg.  John helped Sam to prepare his project.)
	; (eg.  John helped Sam to feed himself.)
)

; TODO avoid checking against noun in the same link and same chunk
(define-method (is-ancient? (nl <nouns-list>) (ni <noun-item>))
	(define index (list-index equal? (slot-ref nl 'lst) (circular-list ni)))
	(define (get-last-time ind)
		(if (< ind 0)
			'()
			; get the noun-item at index 'ind'
			(let ((n (list-ref (slot-ref nl 'lst) ind)))
				(if (and (equal? (get-noun-node n) (get-noun-node ni))			; same noun instance
					 (not (equal? 'InheritanceLink (cog-type (get-orig-link n)))))	; not a modifying link (amod-rule)
					n
					(get-last-time (- ind 1))
				)
			)
		)
	)
	(define last-occurrence (get-last-time (- index 1)))

	; a noun is ancient if appearing more than 3 sentences ago, or not at all
	(or (null? last-occurrence) (< (+ 3 (get-chunk-index last-occurrence)) (get-chunk-index ni)))
)

(define-method (update-pronoun-safety (nl <nouns-list>))
	(define (is-safe? ni)
		(not (or (is-ancient? nl ni) (is-ambiguous? nl ni) (is-modified? nl ni)))	
	)

	(for-each
		(lambda (n) (set-pronoun-safe! n (is-safe? n)))
		(slot-ref nl 'lst)
	)
)

(define-method (populate-nouns-list (nl <nouns-list>) (chunks <list>))
	(define atom-index 0)

	; for each chunk
	(for-each
		(lambda (c chunk-index)
			; for each link in a chunk
			(for-each
				(lambda (link link-index)
					(set! atom-index 0)
					; for each node in a link
					(for-each
						(lambda (node)
							(if (word-inst-is-noun? (r2l-get-word-inst node))
								(add-noun-item
									nl
									(make <noun-item>
										#:noun-node node
										#:orig-link link
										#:atom-index atom-index
										#:link-index link-index
										#:chunk-index chunk-index
									)
								)
							)
							(set! atom-index (+ 1 atom-index))
						)
						(cog-get-all-nodes link)					
					)
				)
				c
				(iota (length c)) ; generate link-indices
			)				
		)
		chunks
		(iota (length chunks)) ; generate chunk-indices
	)

	; determine pronoun safety bases on the current set of chunks
	(update-pronoun-safety nl)
)
