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
; store-referers -- Store to SQL all hypergraphs that contain given atom
;
; This stores all hypergraphs that the given atom participates in.
; It does this by recursively exploring the incoming set of the atom.

(define (store-referers atomo)
	(define (do-store atom)
		(let ((iset (cog-incoming-set atom)))
			(if (null? iset)
				(cog-ad-hoc "store-atom" atom)
				(for-each do-store iset)
			)
		)
	)
	(do-store atomo)
)

; --------------------------------------------------------------------
; load-referers -- Load from SQL all hypergraphs that contain given atom 
;
; This loads all hypergraphs that the given atom participates in.
; It does this by recursively exploring the incoming set of the atom.

(define (load-referers atom)
	(if (not (null? atom))
		; The cog-ad-hoc function for this is defined to perform
		; a recursive fetch.
		; We do an extra recursion here, in case we were passed a list.
		(if (pair? atom)
			(for-each load-referers atom)
			(cog-ad-hoc "fetch-incoming-set" atom)
		)
	)
)

; --------------------------------------------------------------------
; delete-hypergraph -- delete a hypergraph and everything "under" it
;
; If the indicated atom has no incoming links, then delete it. Repeat
; recursively downwards, following the *outgoing* set of any links 
; encountered.

(define (delete-hypergraph atom)
	(if (cog-node? atom) 
		(cog-delete atom)
		(let* ((oset (cog-outgoing-set atom))
				(flg (cog-delete atom))
			)
			(if flg ;; halt recursion if link was not delete-able
				(for-each delete-hypergraph oset)
			)
		)
	)
)

; --------------------------------------------------------------------
; delete-sentence -- delete all atoms associated with a sentence.
;
; Delete the parses and word-instances associated with the sentence,
; including LemmaLink's, ReferenceLinks, RelEx relations, and 
; link-grammar linkages.

(define (delete-sentence sent)
	(define (delete-word-instance wi)
		(cog-delete-recursive wi)
	)

	(define (delete-parse parse)
		(for-each 
			(lambda (x) 
				(if (eq? 'WordInstanceLink (cog-type x))
					(delete-word-instance (car (cog-outgoing-set x)))
				)
			)
			(cog-incoming-set parse)
		)
		(cog-delete-recursive parse)
	)

	(for-each 
		(lambda (x) 
			(if (eq? 'ParseLink (cog-type x))
				(delete-parse (car (cog-outgoing-set x)))
			)
		)
		(cog-incoming-set sent)
	)
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

	; Given a triple trip to some EvaluationLink, walk it down and pull
	; out its word instances. Then pull out its word nodes. Load anything
	; connected to these qord nodes from SQL. Then hunt down the 
	; corresponding SemeNodes, and load those too.
	(define (fetch-seme trip)
		(let* ((w-inst1 (car (cog-outgoing-set (cadr (cog-outgoing-set trip)))))
				(w-inst2 (cadr (cog-outgoing-set (cadr (cog-outgoing-set trip)))))
				(word1 (word-inst-get-lemma w-inst1))
				(word2 (word-inst-get-lemma w-inst2))
			)
			(load-referers word1)
			(load-referers word2)
			(load-referers (cog-chase-link 'LemmaLink 'SemeNode word1))
			(load-referers (cog-chase-link 'LemmaLink 'SemeNode word2))
		)
	)

	; Pull in related stuff for every triple that was created.
	(for-each fetch-seme triple-list)
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
	(define cnt 0)
	(define seme-cnt 0)

	; Get the new input sentences, and run them through the triples processing code.
	; But do it one at a time.
	(define (do-one-sentence sent)
		(attach-sents-for-triple-processing (list sent))

		(set! cnt (+ cnt 1))
		(system (string-join (list "echo start work on sentence " (object->string cnt))))
		(system "date")
		(create-triples)
		(dettach-sents-from-triple-anchor)
		(let ((trip-seme-list (promote-to-seme same-lemma-promoter (get-new-triples))))

			; Print resulting semes to track progress ...
			;;(for-each 
			;;	(lambda (x) 
			;;		(system (string-join (list "echo done triple: \"" (object->string x) "\"")))
			;;	)
			;;	trip-seme-list
  			;;)
  			(set! seme-cnt (+ seme-cnt (length trip-seme-list)))
			(system (string-join (list "echo found  " 
				(object->string (length trip-seme-list)) " triples for a total of "
				(object->string seme-cnt)))
			)

			; XXX we should fetch from SQL ... XXXX
			; (fetch-related-semes trip-seme-list)
			;

			; Save the resulting semes to SQL storage.
			(for-each 
				(lambda (x) 
					; There are *two* semes per triple that need storing.
					(let ((seme-pair (cog-outgoing-set (cadr (cog-outgoing-set x)))))
						(store-referers (car seme-pair))
						(store-referers (cadr seme-pair))
					)
				)
				trip-seme-list
  			)
		)

		; Delete the links to the recently generated triples,
		; and then delete the triples themselves.
		(let ((trip-list (get-new-triples)))
	   	(delete-result-triple-links)
			(for-each delete-hypergraph trip-list)
		)

		; Delete the sentence, its parses, and the word-instances
		(delete-sentence sent)
	)

; xxxxxxxxxx delete the document

	(for-each do-one-sentence (get-new-parsed-sentences))
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
