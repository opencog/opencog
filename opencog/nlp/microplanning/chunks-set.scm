; loading additional dependency
(use-modules (oop goops))

; -----------------------------------------------------------------------
; <chunks-set> -- A container for a set of chunks
;
; One chunks-set is one set of results from processing a list of
; OpenCog links to be said.
;
(define-class <chunks-set> ()
	(chunks #:init-keyword #:chunks #:getter get-chunks)			; the list of chunks, each chunk a list of links
	(uts #:init-keyword #:utterance-types #:getter get-utterance-types)	; the list of utterance types, 1 type for 1 chunk
	(lo #:init-keyword #:leftover-count #:getter get-leftover-count)	; the number of atoms not included in this set
)

; -----------------------------------------------------------------------
; clone-set -- Special copy function to make sure the lists are copied
;
(define-method (clone-set (cs <chunks-set>))
	(make <chunks-set>
		#:chunks (list-copy (slot-ref cs 'chunks))
		#:utterance-types (list-copy (slot-ref cs 'uts))
		#:leftover-count (slot-ref cs 'lo)
	)
)

; -----------------------------------------------------------------------
; get-variation -- Calculate how many times the utterance type changed
;
(define-method (get-variation (cs <chunks-set>))
	(count (lambda (ut1 ut2) (not (string=? ut1 ut2))) (get-utterance-types cs) (cdr (get-utterance-types cs)))
)

; -----------------------------------------------------------------------
; get-length -- Get the length of the list
;
(define-method (get-length (cs <chunks-set>))
	(length (get-chunks cs))
)

; -----------------------------------------------------------------------
; get-chunk -- Get a chunk at "chunk-index"
;
(define-method (get-chunk (cs <chunks-set>) (chunk-index <integer>))
	(list-ref (get-chunks cs) chunk-index)
)

; -----------------------------------------------------------------------
; mod-chunk -- Modify a chunk at "chunk-index"
;
(define-method (mod-chunk (cs <chunks-set>) (chunk-index <integer>) (new-chunk <list>))
	(list-set! (get-chunks cs) chunk-index new-chunk)
)

; -----------------------------------------------------------------------
; get-utterance-type -- Get the utterance type at index
;
(define-method (get-utterance-type (cs <chunks-set>) (index <integer>))
	(list-ref (get-utterance-types cs) index)
)

; -----------------------------------------------------------------------
; get-link -- Get a link at "link-index" of the chunk at "chunk-index"
;
(define-method (get-link (cs <chunks-set>) (chunk-index <integer>) (link-index <integer>))
	(list-ref (get-chunk cs chunk-index) link-index)
)

; -----------------------------------------------------------------------
; mod-link -- Modify a link at "link-index" of the chunk at "chunk-index"
;
(define-method (mod-link (cs <chunks-set>) (chunk-index <integer>) (link-index <integer>) new-link)
	(list-set! (get-chunk cs chunk-index) link-index new-link)
)

; -----------------------------------------------------------------------
; less-leftover? -- Check if the first <chunks-set> has less leftover atoms
;
(define-method (less-leftover? (cs1 <chunks-set>) (cs2 <chunks-set>))
	(< (get-leftover-count cs1) (get-leftover-count cs2))
)

; -----------------------------------------------------------------------
; less-chunks? -- Check if the first <chunks-set> has less number of chunks
;
(define-method (less-chunks? (cs1 <chunks-set>) (cs2 <chunks-set>))
	(< (get-length cs1) (get-length cs2))
)

; -----------------------------------------------------------------------
; less-variation? -- Check if the first <chunks-set> has less variation
;
(define-method (less-variation? (cs1 <chunks-set>) (cs2 <chunks-set>))
	(< (get-variation cs1) (get-variation cs2))
)

; -----------------------------------------------------------------------
; is-subset? -- Check if the first <chunks-set> is the subset of the second
;
(define-method (is-subset? (cs1 <chunks-set>) (cs2 <chunks-set>))
	(define cs1-chunks+utterance-types (zip (get-chunks cs1) (get-utterance-types cs1)))
	(define cs2-chunks+utterance-types (zip (get-chunks cs2) (get-utterance-types cs2)))
	
	(lset<= equal? cs1-chunks+utterance-types cs2-chunks+utterance-types)
)

