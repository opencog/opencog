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
; to its own unique seme, without any checking at all. 
;
; Return the seme itself.

(define (trivial-promoter word-inst)
	(let ((seme (SemeNode (cog-name word-inst) (stv 1 1))))
		(LemmaLink (stv 1 1) seme lemma)
		(InheritanceLink (stv 1 1) word-inst seme)
		(list seme)
	)
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

	(define (make-new-seme wrd-inst lemma)
		(let ((newseme (SemeNode (cog-name wrd-inst) (stv 1 1))))
			(LemmaLink (stv 1 1) newseme lemma)
			(InheritanceLink (stv 1 1) wrd-inst newseme)
			newseme
		)
	)

	(let* ((lemma (word-inst-get-lemma word-inst))
			(seme-list (lemma-get-seme-list lemma))
		)
		(if (null? seme-list)
			; create a new seme
			(list (make-new-seme word-inst lemma))

			; re-use an existing seme
			(let ((seme (car seme-list)))
				(InheritanceLink (stv 1 1) word-inst seme)
				(list seme)
			)
		)
	)
)

; --------------------------------------------------------------------
; Generic seme promoter. 
;
; Given a word inst, and two routines: a new-seme creation routine, and
; a seme matching routine, this will perform all of the other steps
; required for seme promotion in a "generic" way.
;
; The make-new-seme-proc must accept a word instance as its sole argument,
; and return a single new seme. 
;
; The seme-match-proc? must accepet two arguments: a seme and word-inst,
; and return #t if the word-inst can be understood to be an instance of
; the seme. Else it should return #f.
;
; This routine works by creating a list of "candidate semes" for the
; input word-inst. A "candidate seme" is one that has the same lemma as
; the word-inst.  These candidates are then passed through the 
; seme-match-proc? predicate, to locate one or more possible promotions
; for the word-inst.  If any match, these are returned.
;
; If none of the identified "candidate semes" are accepted by the
; predicate seme-match-proc?, then a brand-new seme is created by 
; invoking make-new-seme-proc, and this brand-new seme is returned.
;
; A (relatively) simple example of the use of this promoter can be found in 
; the same-lemma-promoter-two example, below.
;
(define (generic-promoter make-new-seme-proc seme-match-proc? word-instance)

	; We have a list of candidate semes. Are any appropriate?
	; Create one if none are found.
	(define (find-existing-semes seme-list wrd-inst)
		(let ((matching-semes 
					(filter (lambda (se) (seme-match-proc? se wrd-inst)) seme-list))
			)
			(if (null? matching-semes)
				; If list was empty, create a new seme
				(list (make-new-seme-proc wrd-inst))

				; If we got a list of semes, attach each to this word-inst.
				(let ()
					(for-each
						(lambda (seme) 
							(InheritanceLink (stv 1 1) wrd-inst seme)
						)
						matching-semes
					)
					matching-semes
				)
			)
		)
	)

	; Get a list of "candidate semes" having this lemma. 
	(define (lemma-get-seme-list lemma)
		 (cog-chase-link 'LemmaLink 'SemeNode lemma)
	)

	; Get a list of possible, candidate semes for this word-inst
	(define (get-candidate-semes wrd-inst)
		(lemma-get-seme-list (word-inst-get-lemma wrd-inst))
	)

	; Get list of candidate semes, based on thier having a common lemma
	; The vet each of these, to see if one or more provide the desired
	; match. If so, then return them. If not, then create a new seme.
	(define (find-or-make-semes wrd-inst)
		(let* ((seme-list (get-candidate-semes wrd-inst)))
			(if (null? seme-list)
				(list (make-new-seme-proc wrd-inst))
				(find-existing-semes seme-list wrd-inst)
			)
		)
	)

	; Perform an immediate check: this word instance may already
	; be associated with one or more semes. This will not be the 
	; case when encountering a word for the first time, but will 
	; commonly be true when promoting relations. So we add this
	; as a short-cut into the processing path.
	(define (get-existing-semes wrd-inst)
		(cog-chase-link 'InheritanceLink 'SemeNode wrd-inst)
	)

	(let ((exist-semes (get-existing-semes word-instance)))
		(if (null? exist-semes)
			(find-or-make-semes word-instance)
			exist-semes
		)
	)
)

; --------------------------------------------------------------------
; A re-implementation of the same-lemma-promoter, but using the 
; generic-promoter routine. Operationally, this is supposed to return
; exactly the same results as same-lemma-promoter -- see that routine
; for further documentation. The point is that this implementation 
; should be easier to understand and debug, since it just implements
; the two basic routines needed by the generic promoter.
;
(define (same-lemma-promoter-two word-inst)

	; Create a new seme corresponding to this word-instance.
	(define (make-new-seme wrd-inst)
		(let ((newseme (SemeNode (cog-name wrd-inst) (stv 1 1)))
				(lemma (word-inst-get-lemma wrd-inst))
			)
			(LemmaLink (stv 1 1) newseme lemma)
			(InheritanceLink (stv 1 1) wrd-inst newseme)
			newseme
		)
	)

	; If the seme and the word inst have the same lemma,
	; then they match.
	(define (match-seme? seme wrd-inst)
		(equal?
			(word-inst-get-lemma seme)
			(word-inst-get-lemma wrd-inst)
		)
	)

	; Use the generic routine.
	(generic-promoter make-new-seme match-seme? word-inst)
)

; --------------------------------------------------------------------
; noun-same-modifiers-promoter -- re-use an existing seme if it has a 
; superset of the modifiers of the word instance. Otherwise, create a 
; new seme. DEPRECATED -- USE same-dependency-modifier BELOW. The reason
; that this is deprecated is because it only works correctly for nouns;
; whereas it will alias together all verbs, which is incorrect for verbs!
;
; The idea here is that if we have a word instance, such as "ball", and
; a seme "green ball", we can deduce "oh the ball, that must be the 
; green ball".  But if the word-instance is "red ball", then it cannot
; be the seme "green ball", and a new seme, specific to "red ball" is
; created. 
;
; Specifically, we try to make sure that *every* modifier to the word-inst
; is also a modifier to the seme. i.e. that the modifiers on the word-inst
; are a subset of the modifiers on the seme. i.e. that the word-inst is 
; "semantically broader" than the seme.  This idea is discussed in greater
; detail in the README file.
;
(define (noun-same-modifiers-promoter word-inst)

	; Create a new seme, given a word-instance. The new seme will 
	; have the same modifiers that the word-instance has.
	(define (make-new-seme wrd-inst)
		(let* ((newseme (SemeNode (cog-name wrd-inst) (stv 1 1)))
				(lemma (word-inst-get-lemma wrd-inst))
				(mods (noun-inst-get-relex-modifiers wrd-inst))
			)
			; Be sure to create the inheritance link, etc. before
			; doing the promotion.
			(LemmaLink (stv 1 1) newseme lemma)
			(InheritanceLink (stv 1 1) wrd-inst newseme)
			(promote-to-seme noun-same-modifiers-promoter mods)
			newseme
		)
	)

	; Given a seme, and a "modifier relation" mod-rel of the form:
	;    EvaluationLink
	;       prednode (a DefinedLinguisticPredicateNode)
	;       ListLink
	;          headword   (a WordInstanceNode)
	;          attr-word  (a WordInstanceNode)
	; this routine checks to see if the corresponding relation
	; exists for seme. If it does, it returns #t else it returns #f
	;
	(define (does-seme-have-rel? seme mod-rel)
		(let* ((oset (cog-outgoing-set mod-rel))
				(prednode (car oset))
				(attr-word (cadr (cog-outgoing-set (cadr oset))))
				(attr-seme (noun-same-modifiers-promoter attr-word))
				(seme-rel (cog-link 'EvaluationLink prednode (ListLink seme attr-seme)))
			)
			(if (null? seme-rel) #f #t)
		)
	)

	; Could this noun word-inst correspond to this seme?
	; It does, if *every* modifier to the word-inst is also a
	; modifier to the seme. i.e. if the modifiers on the word-inst
	; are a subset of the modifiers on the seme. i.e. if the
	; word-inst is "semantically broader" than the seme.  Thus,
	; the word-inst "ball" matches the seme "green ball".
	(define (noun-seme-match? seme wrd-inst)
		(every 
			(lambda (md) (does-seme-have-rel? seme md)) 
			(noun-inst-get-relex-modifiers wrd-inst)
		)
	)
	(generic-promoter make-new-seme noun-seme-match? word-inst)
)

; --------------------------------------------------------------------
; same-dependency-promoter -- re-use an existing seme if it has a 
; superset of the modifiers of the word instance. Otherwise, create a 
; new seme.  This performs seme promotion for both nouns and verbs,
; looking for noun-modifiers (e.g. _amod) on nouns and looking at 
; subject, object, adverb relations for verbs.
;
; The idea here is that if we have a word instance, such as "ball", and
; a seme "green ball", we can deduce "oh this ball, it must refer to the 
; green ball".  But if the word-instance is "red ball", then it cannot
; be the seme "green ball", and a new seme, specific to "red ball" is
; created. 
;
; Specifically, we try to make sure that *every* modifier to the word-inst
; is also a modifier to the seme. i.e. that the modifiers on the word-inst
; are a subset of the modifiers on the seme. i.e. that the word-inst is 
; "semantically broader" than the seme.  
;
; This is a fairly basic operation, and lacks in many ways: we'd like 
; to do this only for recent words in the conversation, and we'd also like
; to do narrowing, e.g. so if we get "John threw the ball. John threw the 
; blue ball.", we conclude that the ball in the second sentence is the same
; as that in the first. The routine fails to handle this situation.  This
; routine should probably not be "fixed", and instead, a new, more 
; sophisticated promoter should be created.
;
; Anyway, seme promotion should not be done in scheme, but with opencog
; pattern-matching. So, for example, the following ImplicationLink is a 
; step in that direction:
;
; IF   %InheritanceLink(word-inst $word-seme)
;    ^ $modtype (word-inst, $attr-inst)
;    ^ $modtype is _amod or _nn etc.
;    ^ %InheritanceLink($attr-inst $attr-seme)
;    ^ $modtype ($seme, $attr-seme)
;    ^ $seme is a SemeNode
; THEN $modtype($seme, $attr-seme)
;
; When promoting verbs, this promoter is careful to handle the HYP
; and TRUTH-QUERY_FLAG as well.  The README file goes into more detail
; on the how and why of handling these two flags.
;
; As of mid-September 2009, this is working well, and doing the right 
; thing for general question answering.
;
(define (same-dependency-promoter word-instance)

	; Return a list of all of the relex relationships for which this
	; word instance is the head-word.
	(define (get-relex-rels wrd-inst)
		(cond
			((word-inst-is-noun? wrd-inst) 
				(append! 
					(noun-inst-get-relex-modifiers wrd-inst)
					(noun-inst-get-prep-rels wrd-inst)
				)
			)
			((word-inst-is-verb? wrd-inst) (verb-inst-get-relex-rels wrd-inst))
			(else '())
		)
	)

	; Given an InheritanceLink 'link', returns #t if the link has a 
	; truth-query or hypothetical attribute.
	(define (is-truqu? link)
		(define (is-truqu-str? str)
			(or
				(string=? "truth-query" str)
				(string=? "hyp" str)
			)
		)
		(define (is-truqu-node? node)
			(and
				(eq? 'DefinedLinguisticConceptNode (cog-type node))
				(is-truqu-str? (cog-name node))
			)
		)
		(is-truqu-node? (cadr (cog-outgoing-set link)))
	)

	; Return a list of InheritenceLinks containing the HYP and
	; TRUTH-QUERY flags, if present.  This list will be promoted, below.
	(define (get-truqu-flags wrd-inst)
		(filter! is-truqu?
			(cog-filter 'InheritanceLink (cog-incoming-set wrd-inst))
		)
	)

	; Don't bother looking for truth query flags unless its a verb
	(define (get-truqu-flags-for-verb wrd-inst)
		(if (word-inst-is-verb? wrd-inst)
			(get-truqu-flags wrd-inst)
			'()
		)
	)

	; Create a new seme, given a word-instance. The new seme will 
	; have the same modifiers that the word-instance has.
	; Since this is used for promoting words in questions as well, 
	; we need to promote the HYP and TRUTH-QUERY flags as well.
	(define (make-new-seme wrd-inst)
		(let* ((newseme (SemeNode (cog-name wrd-inst) (stv 1 1)))
				(lemma (word-inst-get-lemma wrd-inst))
				(rels (get-relex-rels wrd-inst))
				(flgs (get-truqu-flags-for-verb wrd-inst))
			)
			; Be sure to create the inheritance link, etc. before
			; doing the promotion.
			(LemmaLink (stv 1 1) newseme lemma)
			(InheritanceLink (stv 1 1) wrd-inst newseme)
			(promote-to-seme same-dependency-promoter rels)
			(promote-to-seme same-dependency-promoter flgs)
			newseme
		)
	)

	; Given a seme, and a relex relation rel of the form:
	;    EvaluationLink
	;       prednode (a DefinedLinguisticPredicateNode)
	;       ListLink
	;          headword   (a WordInstanceNode)
	;          attr-word  (a WordInstanceNode)
	; This routine checks to see if the corresponding relation
	; exists for seme. If it does, it returns #t else it returns #f
	;
	(define (does-seme-have-rel? seme rel)
		(let* ((oset (cog-outgoing-set rel))
				(prednode (car oset))
				(dependent-word (cadr (cog-outgoing-set (cadr oset))))
				(dependent-seme (same-dependency-promoter dependent-word))
				(seme-rel (cog-link 'EvaluationLink prednode (ListLink seme dependent-seme)))
			)
			(if (null? seme-rel) #f #t)
		)
	)

	; Could this noun word-inst correspond to this seme?
	; It does, if *every* modifier to the noun word-inst is also a
	; modifier to the seme. i.e. if the modifiers on the word-inst
	; are a subset of the modifiers on the seme. i.e. if the
	; word-inst is "semantically broader" than the seme.  Thus,
	; the word-inst "ball" matches the seme "green ball".
	(define (noun-seme-match? seme wrd-inst)
		(every 
			(lambda (md) (does-seme-have-rel? seme md)) 
			(noun-inst-get-relex-modifiers wrd-inst)
		)
	)

	; As above, but for verbs. In particular, the subject and
	; object of the verb must match.
	(define (verb-seme-match? seme wrd-inst)
		(every 
			(lambda (md) (does-seme-have-rel? seme md)) 
			(verb-inst-get-relex-rels wrd-inst)
		)
	)

	(define (xor a b) (or (and a b) (not (or a b))))

	; As above, but also checks for truth-query markings.
	; A word-inst can be promoted to a seme if:
	; -- both have HYP/TRUTH-QUERY markings
	; -- neither have HYP/TRUTH-QUERY markings
	(define (qu-verb-seme-match? seme wrd-inst)
		(and
			(verb-seme-match? seme wrd-inst)
			(xor
				(null? (get-truqu-flags seme))
				(null? (get-truqu-flags wrd-inst))
			)
		)
	)

	; For anything that's not a noun or a verb, all that we ask for
	; is that it has the same word lemma. This seems safe for now,
	; but will go bad if there's a chain of noun-adj-noun-adj modifiers
	; such as those in medical text.  This might fail for chained
	; adverbial modifers (?not sure?)
	(define (same-lemma-match? seme wrd-inst)
		(equal?
			(word-inst-get-lemma seme)
			(word-inst-get-lemma wrd-inst)
		)
	)

	; Return #t if the word inst could be promoted to the seme, else
	; return #f.
	(define (seme-match? seme wrd-inst)
		(cond
			((word-inst-is-noun? wrd-inst) (noun-seme-match? seme wrd-inst))
			((word-inst-is-verb? wrd-inst) (qu-verb-seme-match? seme wrd-inst))
			(else (same-lemma-match? seme wrd-inst))
		)
	)

	(generic-promoter make-new-seme seme-match? word-instance)
)

(define (same-modifiers-promoter word-inst)
	(same-dependency-promoter word-inst)
)

; --------------------------------------------------------------------
;
; promote-to-seme -- promote all WordInstanceNodes to SemeNodes
;
; Given a specific promotor, and a list of hypergraphs, this routine
; will walk over all the hypergraphs, find every WordInstanceNode, call
; the promoter on it to get one or more SemeNodes, and then construct a 
; brand-new hypergraph with each SemeNode taking the place of the 
; WordInstanceNode.  
;
; Returns the list of the equivalent, promoted relations.

(define (promote-to-seme promoter atom-list)

	(define (promote atom)
		; Returns a list of one or more different possible promotions
		; of a link.  If eny elt in the outgoing set can be promoted 
		; in more than one way, then the returned list will have more 
		; than one link in it.
		(define (promo-link lnk)

			; Given a link and an outgoing set, make a new link of the 
			; same type and TV, but with the given outset.
			(define (mklink lnk oset)
				(cog-new-link (cog-type lnk) oset (cog-tv lnk))
			)

			; Note the recursion in bowles of this below.
			(map 
				(lambda (oset) (mklink lnk oset)) 
				; The Cartesian prod will distribute the list of lists.
				(cartesian-prod 
					; Applying map will generate a list of lists
					(map promote (cog-outgoing-set lnk))
				)
			)
		)

		(cond
			((eq? 'WordInstanceNode (cog-type atom))
				(promoter atom)
			)
			((cog-link? atom)
				(promo-link atom)
			)
			(else atom)
		)
	)

	(concatenate! (map promote atom-list))
)

; --------------------------------------------------------------------
;
; fetch-related-semes -- get semes from persistant storage.
;
; Fetch, from persistant (SQL) storage, all knowledge related to the
; recently produced triples. Specifically, hunt out the SemeNode's that
; occur in the triples, and get everything we know about them (by getting
; everything that has that seme-node in it's outgoing set.) Basically,
; the goal is to get stuff out of persistent store and into RAM.
;
; The input triples are presumed to be expressed in terms of
; word-instances. This routine looks up the words, and then any semes
; associated with that word.
; 
(define (fetch-related-semes triple-list)

	; Given a triple trip to some EvaluationLink, walk it down and pull
	; out its word instances. Then pull out its word nodes. Load anything
	; connected to these qord nodes from SQL. Then hunt down the 
	; corresponding SemeNodes, and load those too.
	(define (fetch-seme trip)
		(let* ((wrd-inst1 (car (cog-outgoing-set (cadr (cog-outgoing-set trip)))))
				(wrd-inst2 (cadr (cog-outgoing-set (cadr (cog-outgoing-set trip)))))
				(word1 (word-inst-get-lemma wrd-inst1))
				(word2 (word-inst-get-lemma wrd-inst2))
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
		(let* ((trip-list (get-new-triples))
			 	(trip-seme-list (promote-to-seme same-lemma-promoter trip-list)))

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

			; Delete the links to the recently generated triples,
			; and then delete the triples themselves.
			(release-result-triples)
			(for-each delete-hypergraph trip-list)
		)

		; Delete the sentence, its parses, and the word-instances
		(delete-sentence sent)

		; Delete upwards ... this deletes the link to the document,
		; and also the link to the new-parsed-sentences anchor.
		; XXX but it leaves a DocumentNode with nothing pointing to it.
		(cog-delete-recursive sent)
	)

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
(define (do-implication x) #t)  ; XXX use cog-bind instead ... 
(define (xxxprocess-rule rule)
	(define triple-list (cog-outgoing-set (do-implication rule)))

	; Increment count by 1 on each result.
	(for-each (lambda (atom) (cog-atom-incr atom 1)) triple-list)
	; (system "date")
	; (system "echo Done running one implication\n")

	; Store each resultant atom.
	(for-each (lambda (atom) (store-atom atom)) triple-list)
)

