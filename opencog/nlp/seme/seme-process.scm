scm
;
; seme-process.scm
;
; Perform seme processing
;
; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
; trivial-promoter -- promote WordInstanceNode to a SemeNode
;
; Given a word instance node, returns a corresponding seme node.
; The promotion is "trivial", in that ever word instance is converted
; to a seme, without any checking at all.

(define (trivial-promoter word-inst)
	(SemeNode (cog-name word-inst) (stv 1 1))
)

; --------------------------------------------------------------------
; same-lemma-promoter -- promote to seme, based on the associated lemma.
;
; Given a word instance, compare the WordNode associated with the 
; instance to the WordNode of a SemeNode. Return the first SemeNode
; found; if not found, create a new SemeNode with this word.  So, for
; example, given the existing link
;     LemmaLink 
;         SemeNode "house@a0a2"
;         WordNode "house"
;
; and the input WordInstanceNode "house@45678", it will return the 
; SemeNode "house@a0a2", since it has the same lemma.

(define (same-lemma-promoter word-inst)

	; Get a list of semes with this lemma. 
	(define (lemma-get-seme-list lemma)
		 (cog-chase-link 'LemmaLink 'SemeNode lemma))

	(let* ((lemma (word-inst-get-lemma word-inst))
			(seme-list (lemma-get-seme-list lemma))
		)
		(if (null? seme-list)
			(let ((newseme (SemeNode (cog-name word-inst) (stv 1 1))))
				(LemmaLink newseme lemma (stv 1 1))
				newseme
			)
			(car seme-list)
		)
	)
)

;
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
; store-seme -- Save semes to SQL database
;
; This saves all hypergraphs that a given SemeNode participates in.
; It does this by recursively exploring the incoming set of the seme.

(define (store-seme seme)
	(define (do-store atom)
		(let ((iset (cog-incoming-set atom)))
			(if (null? iset)
				(cog-ad-hoc "store-atom" atom)
				(for-each do-store iset)
			)
		)
	)
	(do-store seme)
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
		(let ((seme-list (promote-to-seme same-lemma-promoter (get-new-triples))))
			(define (trip-seme trip)
				(car (cog-outgoing-set (cadr (cog-outgoing-set trip))))
			)

			; Print resulting semes to track progress ,,, 
			(for-each 
				(lambda (x) 
					(system (string-join (list "echo done triple: \"" (object->string x) "\"")))
				)
				seme-list
  			)

			; XXX we should fetch from SQL ... 
			; (fetch-related-semes seme-list)
			;

			; Save the resulting semes to SQL storage.
			(for-each 
				(lambda (x) (store-seme (trip-seme x)))
				seme-list
  			)
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
