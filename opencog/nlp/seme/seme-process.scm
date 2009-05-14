scm
;
; seme-process.scm
;
; Perform seme processing
;
; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
; 
; trivial-promoter -- promote WordInstanceNode to a SemeNode
;
; Given a word instance node, returns a corresponding seme node.
; The promotion is "trivial", in that ever word instance is converted
; to a seme, without any checking at all.

(define (trivial-promoter word-inst)
	(SemeNode (cog-name word-inst) (stv 1 1))
)

; --------------------------------------------------------------------
;
; promote-to-seme -- promote all WordInstanceNodes to SemeNodes
;
; Given a specific promotor, and a list of hypergraphs, this routine
; will walk over all the hyprgraphs, find every WordInstanceNode, call
; the promoter on it to get a SemeNode, and then construct a brand-new
; hypergraph with the SemeNode taking the place of the WordInstanceNode.

(define (promote-to-seme promoter atom-list)
	(define (promote atom)
		(if (eq? 'WordInstanceNode (cog-type atom))
			(promoter atom)
			(if (cog-link? atom)
				(cog-new-link 
					(cog-type atom) 
					(map promote (cog-outgoing-set atom))
					(cog-tv atom)
				)
				atom
			)
		)
	)

	(map promote atom-list)
)

; --------------------------------------------------------------------
;
; fetch-related-semes -- get semes from persistant storage.
;
; Fetch, from persistant (SQL) storage, all knowledge related to the
; recently produced triples. Specifically, hunt out the SemeNode's that
; occur in the triples, and get everything we know about them (by getting
; everything that has that seme-node in it's outgoing set.)
; 
(define (fetch-related-semes triple-list)

	; Given a handle h to some EvaluationLink, walk it down and pull
	; in any related SemeNode expressions.
	; XXX I think this is broken since it doesn't recurse up the incoming set ...
	(define (fetch-seme h)
		(if (eq? 'SemeNode (cog-type h))
			(cog-ad-hoc "fetch-incoming-set" h)
		)
		(for-each fetch-seme (cog-outgoing-set h))
	)

	; Pull in related stuff for every triple that was created.
	(for-each fetch-word triple-list)
)

; --------------------------------------------------------------------
; 
; do-seme-processing -- ad-hoc routine under development.
;
; Process parsed text through the prepositional-triples code.
;
; This will run the preposition-triple rules through the forward
; chainer, and then go through the results, updating the 
; CountTruthValue associated with each, and then storing the 
; updated count in the OpenCog persistence backend.
;

(define (do-seme-processing)

	; Get the new input sentences, and run them through the triples processing code.
	; But do it one at a time.
	(define (do-one-sentence sent)
		(attach-sents-for-triple-processing (list sent))
		(system "echo start work an a triple")
		(system "date")
		(create-triples)
		(dettach-sents-from-triple-anchor)
		(let ((seme-list (promote-to-seme trivial-promoter (get-new-triples))))
			(for-each 
				(lambda (x) 
					(system (string-join (list "echo done triple: \"" (object->string x) "\"")))
				)
	      	seme-list
  			)
			; (fetch-related-semes seme-list)
		)
	   (delete-result-triple-links)
	)

	(for-each do-one-sentence (get-new-parsed-sentences))

	;(delete-sentences)
)


; XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
;
; dead code needs to be resurrected.
; process-rule -- apply an ImplicationLink
;
; Given an ImplicationLink, apply the implication on the atom space.
; This may generate a list of atoms. Take that list, and manually
; store it in the database.
;
(define (xxxprocess-rule rule)
	(define triple-list (cog-outgoing-set (cog-ad-hoc "do-implication" rule)))

	; Increment count by 1 on each result.
	(for-each (lambda (atom) (cog-atom-incr atom 1)) triple-list)
	; (system "date")
	; (system "echo Done running one implication\n")

	; Store each resultant atom.
	(for-each (lambda (atom) (cog-ad-hoc "store-atom" atom)) triple-list)
)

.
exit
