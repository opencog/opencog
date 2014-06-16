;
; nlp-utils.scm
;
; Assorted NLP utilities.  Operations include:
; -- looping over all sentences in a document
; -- looping over all parses of a sentence (map-parses)
; -- looping over all words in a sentence (map-word-node)
; -- looping over all RelEx relations
; -- get word of word instance. (word-inst-get-word)
; -- get part-of-speech, lemma of word.
; -- get prepositions
; -- get word senses
; -- deleting all atoms pertaining to a sentence
;
; The function names that can be found here are:
; -- map-parses   Call proceedure on every parse of the sentence.
; -- map-word-instances   Call proc on each word-instance of parse.
; -- map-word-node        Call proc on the word-node associated to word-inst.
; -- document-get-sentences Get senteces in document.
; -- sentence-get-parses    Get parses of a sentence.
; -- sent-list-get-parses   Get parses of a list of sentences.
; -- parse-get-words        Get all words occuring in a parse.
; -- parse-get-words-in-order  Get all words occuring in a parse in order.
; -- parse-get-relations    Get all RelEx relations in a parse.
; -- word-inst-get-number   Return the NumberNode associated with word-inst. 
; -- word-inst-get-word   Return the WordNode associated with word-inst.
; -- word-inst-get-word-str  Return the word string assoc with word-inst.
; -- word-inst-get-lemma  Return the lemma of word instance.
; -- word-inst-get-attr   Return attributes of word instance.
; -- word-inst-get-pos    Return part-of-speech (POS) of word instance.
; -- word-inst-match-pos? Does word-inst have POS?
; -- word-inst-is-noun?   Is word instance a noun?
; -- word-inst-is-verb?   Is word instance a verb?
; -- word-inst-get-relations       Get RelEx relations involving word-inst.
; -- word-inst-get-head-relations  Get relations with word-inst as head.
; -- word-inst-get-prep-relations  Get prepositional relations for word-inst.
; -- word-inst-filter-relex-rels   Get filtered set of RelEx relations.
; -- verb-inst-get-relex-rels      Get relations for a verb.
; -- noun-inst-get-relex-modifiers Get relations for a noun.
; -- noun-inst-get-prep-rels       Get prep relations for noun.
; -- word-inst-get-subscript-str   Get link-grammar subscript for word-inst.
; -- word-inst-get-subscripted-word-str Get LG subscripted word string.
; -- word-inst-get-senses   Get word senses associated with word.
; -- word-inst-sense-score  Get ranking score for word-inst & word-sense.
; -- relation-get-dependent Get dependent part of a relation.
; -- delete-sentence        Delete all atoms associated with sentence.
; -- delete-sentences       Delete all atoms that occur in sentences.
;
;
; Important Design Note: A long-term goal of NLP within opencog is to
; do processing not in scheme, but in OpenCog itself, using pattern
; matching. The reason for this is so that we can apply OpenCog learning
; algos to learn new ways of processing. Many/most of the utilities below
; could be implemented by using pattern maching. Code that depends on these
; utilities should be converted to use pattern matching as soon as reasonable.
; Code that cannot be converted will eventually (in the distant future ...) 
; become obsolete.
; 
; Copyright (c) 2008, 2009, 2013 Linas Vepstas <linasvepstas@gmail.com>
;
; ---------------------------------------------------------------------
; map-parses   Call proceedure on every parse of the sentence.
;
; map-parses proc sent
; Call proceedure 'proc' on every parse of the sentence 'sent' 
; 
; Expected input is a SentenceNode, or possibly a list of SentenceNodes.
; Each SentenceNode serves as an anchor to all of the parses of a sentence.
; It is connected via ParseLink's to each of the individual parses of the
; sentence. This routine backtracks over the ParseNode to find these.
;
; The recursion will stop if proc returns something other than #f. This
; routine returns the last value that stopped the recursion. (In other
; words, this is not really a map, but something kind-of weird -- XXX
; this should probably be fixed -- TODO)
;
(define (map-parses proc sent-or-list)
	(cog-map-chase-links-chk 'ParseLink 'ParseNode
		proc sent-or-list 'SentenceNode)
)

; Same as above, but multi-threaded -- each parse dispatched to its own
; thread, on a distinct CPU.
(define (parallel-map-parses proc sent-or-list)
	(cog-par-chase-links-chk 'ParseLink 'ParseNode
		proc sent-or-list 'SentenceNode)
)

; ---------------------------------------------------------------------
; map-word-instances   Call proc on each word-instance of parse.
;
; map-word-instances proc parse
; Call proceedure 'proc' on each word-instance of 'parse'
;
; Expected input is a ParseNode or a list of ParseNodes. These serve
; as anchors to all of the word instances in a parse. The word instances
; can be found by back-tracking through the WordInstanceLink to the
; individual words, which is what this method does.
; 
(define (map-word-instances proc parse-or-list) 
	(cog-map-chase-links-chk 'WordInstanceLink 'WordInstanceNode
		proc parse-or-list 'ParseNode)
)

; ---------------------------------------------------------------------
; map-word-node        Call proc on the word-node associated to word-inst.
;
; map-word-node proc word-inst
; Call proceedure 'proc' on the word-node associated to 'word-inst'
;
; Expected input is a WordInstanceNode, which serves as an anchor
; to a word instance. The WordInstanceNode is connnected via a ReferenceLink
; to the actual word node.
;
(define (map-word-node proc word-inst) 
	(cog-map-chase-links-chk 'ReferenceLink 'WordNode
		proc word-inst 'WordInstanceNode)
)

; ---------------------------------------------------------------------
; document-get-sentences Get senteces in document.
;
; document-get-sentences doco
;
; Given a document, return a list of sentences in that document
; Throws an error if doco is not a DocumentNode
;
(define (document-get-sentences doco)
	(if (eq? (cog-type doco) 'DocumentNode)
		(cog-get-reference doco)
		(throw 'wrong-atom-type 'document-get-sentences
			"Error: expecting DocumentNode:" doco)
	)
)

; ---------------------------------------------------------------------
; sentence-get-parses    Get parses of a sentence.
;
; sentence-get-parses sent-node
;
; Given a sentence, return a list of parses in that sentence
; Basically, chase a ParseLink to a ParseNode
; Throws an error if sent-node is not a SentenceNode
;
(define (sentence-get-parses sent-node)
	(cog-chase-link-chk 'ParseLink 'ParseNode sent-node 'SentenceNode)
)

; -----------------------------------------------------------------------
; sent-list-get-parses   Get parses of a list of sentences.
;
; Given a list of sentences, return a list of parses of those sentences.
; That is, given a List of SentenceNode's, return a list of ParseNode's
; associated with those sentences.

(define (sent-list-get-parses sent-list)
	(concatenate! (map sentence-get-parses sent-list))
)


; ---------------------------------------------------------------------
; parse-get-words - Given a parse, return a list of all words in the parse
;
; Given a parse, return all word instances in arbitary order, keeping LEFT-WALL
; This version is faster than the in order version.
;
(define (parse-get-words parse-node)
	(cog-chase-link 'WordInstanceLink 'WordInstanceNode parse-node)
)

; ---------------------------------------------------------------------
; parse-get-words-in-order - Given a parse, return a list of all words in the parse in order
;
; Given a parse, return all word instances in order, skipping LEFT-WALL
;
(define (parse-get-words-in-order parse-node)
	(define word-inst-list (cog-chase-link 'WordInstanceLink 'WordInstanceNode parse-node))
	(define number-list (map word-inst-get-number word-inst-list))
	(define (less-than word-inst-1 word-inst-2)
		(define index-1 (list-index (lambda (a-node) (equal? word-inst-1 a-node)) word-inst-list))
		(define index-2 (list-index (lambda (a-node) (equal? word-inst-2 a-node)) word-inst-list))
		(< (string->number (cog-name (list-ref number-list index-1))) (string->number (cog-name (list-ref number-list index-2)))))
	(cdr (sort word-inst-list less-than))
)

; --------------------------------------------------------------------
; parse-get-relations    Get all RelEx relations in a parse.
;
; Given a parse, return a list of all relex relations
;
(define (parse-get-relations parse-node)
	; Get a list of words in the parse
	; Get a list of lists of relations for each word
	; Conctenate will reduce the list of lists to a plain list
	; Remove duplicates
	(delete-duplicates!
		(concatenate!
			(map word-inst-get-relations 
				(parse-get-words parse-node)
			)
		)
	)
)

; ---------------------------------------------------------------------
; word-inst-get-number   Return the NumberNode associated with word-inst
;
; Return the NumberNode associated with 'word-inst'
;
(define (word-inst-get-number word-inst)
	(car (cog-chase-link 'WordSequenceLink 'NumberNode word-inst))
)

; ---------------------------------------------------------------------
; word-inst-get-word   Return the WordNode associated with word-inst
;
; Return the WordNode associated with 'word-inst'
;
(define (word-inst-get-word word-inst)
	(cog-chase-link 'ReferenceLink 'WordNode word-inst)
)

; ---------------------------------------------------------------------
; word-inst-get-word-str  Return the word string assoc with word-inst 
;
; Return the word string associated with the word-instance
(define (word-inst-get-word-str word-inst)
	(cog-name (car (word-inst-get-word word-inst)))
)

; ---------------------------------------------------------------------
; word-inst-get-lemma  Return the lemma of word instance.
;
; Given a word instance, return the lemma for of the word.
; Also works if the word-inst is actually a seme.
(define (word-inst-get-lemma word-inst)
	(let ((wlist (cog-chase-link 'LemmaLink 'WordNode word-inst)))
		(if (null? wlist)
			'()
			(car wlist)
		)
	)
)

; ---------------------------------------------------------------------
; word-inst-get-attr   Return attributes of word instance
;
; Given a word instance, return a list of attributes for the word.
;
(define (word-inst-get-attr word-inst)
	(cog-chase-link 'InheritanceLink 'DefinedLinguisticConceptNode 
		word-inst
	)
)

; ---------------------------------------------------------------------
; word-inst-get-pos    Return part-of-speech (POS) of word instance
;
; Return the part-of-speech of a word-instance
; 
(define (word-inst-get-pos word-inst)
	(cog-chase-link 'PartOfSpeechLink 'DefinedLinguisticConceptNode 
		word-inst
	)
)

; ---------------------------------------------------------------------
; word-inst-match-pos? Does word-inst have POS?
;
; Return #t is the word-instance has part-of-speech pos else #f
; "word-inst" must have a PartOfSpeechLink on it; typically its 
; a WordInstanceNode.
; "pos" is a string, typically "noun" or "verb".
;
; Not all word instances are always tagged. For example, in a sentence
; with "John Kennedy", John won't have a pos tag, because RelEx will
; have already meged this into an entity string (and tagged the string).
;
(define (word-inst-match-pos? word-inst pos)
	(let ((pos-list (word-inst-get-pos word-inst)))
		(if (null? pos-list)
			#f
			(string=? pos (cog-name (car pos-list)))
		)
	)
)

; ---------------------------------------------------------------------
; word-inst-is-noun?   Is word instance a noun?
;
; Return #t is the word-instance is a noun
; 
(define (word-inst-is-noun? word-inst)
	(word-inst-match-pos? word-inst "noun")
)

; ---------------------------------------------------------------------
; word-inst-is-verb?   Is word instance a verb?
;
; Return #t is the word-instance is a verb
; 
(define (word-inst-is-verb? word-inst)
	(word-inst-match-pos? word-inst "verb")
)

; ---------------------------------------------------------------------
; word-inst-get-relations       Get RelEx relations involving word-inst
;
; Given a word instance, return a list of relations the word participates in.
; That is, given (WordInstanceNode "dog"), return all _subj(*,dog)
; _subj(dog,*), _obj(dog,*) etc.
;
; See also word-inst-get-head-relations and 
; word-inst-get-relex-modifiers below.
;
(define (word-inst-get-relations word-inst)
	(cog-get-pred word-inst 'DefinedLinguisticRelationshipNode)
)

; ---------------------------------------------------------------------
; word-inst-get-prep-relations  Get prepositional relations for word-inst
;
(define (word-inst-get-prep-relations word-inst)
	(cog-get-pred word-inst 'PrepositionalRelationshipNode)
)

; --------------------------------------------------------------------
; word-inst-get-head-relations  Get relations with word-inst as head
;
; Given a word-instance, return a list of relex relations that 
; this word-instance is the head-word of. (It is the head-word
; if it is first e.g. _amod(head-word, attr-word)
;
(define (word-inst-get-head-relations word-inst)

	; Return #t if the word-inst is the head-word.
	(define (head? rel wrd-inst)
		(let* ((oset (cog-outgoing-set rel))
				(head (car (cog-outgoing-set (cadr oset))))
			)
			(equal? head wrd-inst)
		)
	)

	; Get all relations, and filter them out.
	(filter! 
		(lambda (rel) (head? rel word-inst))
		(word-inst-get-relations word-inst)
	)
)

; --------------------------------------------------------------------
; word-inst-filter-relex-rels   Get filtered set of RelEx relations
;
; Given a word-instance and a list of relex relation names, return a
; list of the relex relations for which this word-instance is the 
; head-word.  The head-word is the first word in the relation -- 
; for example _amod(head-word, dependent-word)

(define (word-inst-filter-relex-rels word-inst rel-name-list)

	(define (is-modifier? str)
		(any (lambda (rel) (string=? rel str)) rel-name-list)
	)

	; Return #t if the relex relation is in the rel-name-list
	; and the word-inst is the head-word.
	(define (head-mod? rel wrd-inst)
		(let* ((oset (cog-outgoing-set rel))
				(head (car (cog-outgoing-set (cadr oset))))
			)
			(and 
				(equal? head wrd-inst)
				(is-modifier? (cog-name (car oset)))
			)
		)
	)

	; Get all relations, and filter them out.
	(filter! 
		(lambda (rel) (head-mod? rel word-inst))
		(word-inst-get-relations word-inst)
	)
)

; --------------------------------------------------------------------
; verb-inst-get-relex-rels      Get relations for a verb.
;
; Given a verb word-instance, return a list of relex relations that 
; this word-instance is the head-word of. (It is the head-word
; if it is first e.g. _subj(head-word, dependency-word)
;
(define (verb-inst-get-relex-rels word-inst)
	; There are a few other relations we should probably deal with,
	; e.g. _predadj Right now, this is unclear.
	; Modifiers are documented at:
	; http://opencog.org/wiki/Binary_relations
	(word-inst-filter-relex-rels word-inst 
		(list "_advmod" "_iobj" "_obj" "_subj")
	)
)

; --------------------------------------------------------------------
; noun-inst-get-relex-modifiers Get relations for a noun.
;
; Given a noun word-instance, return a list of relex modifiers that 
; this word-instance is the head-word of. (It is the head-word
; if it is first e.g. _amod(head-word, attr-word)
;
; This very explicitly returns *only* modifiers, and not relations in
; general. 
;
(define (noun-inst-get-relex-modifiers word-inst)
	; There are a few other modifiers we should probably deal with,
	; including quantity multiplier, etc. Right now, this is unclear.
	; Modifiers are documented at:
	; http://opencog.org/wiki/Binary_relations
	(word-inst-filter-relex-rels word-inst 
		(list "_amod" "_appo" "_nn" "_poss" "_%quantity")
	)
)

; --------------------------------------------------------------------
; noun-inst-get-prep-rels       Get prep relations for noun.
;
; Given a noun word-instance, return a list of prepositional relations
; that this word-instance is the head-word of. (It is the head-word
; if it is first e.g. prep(head-word, dependent-word)
;
(define (noun-inst-get-prep-rels word-inst)

	; Return #t if the relex relation is prepositional,
	; and the word-inst is the head-word.
	(define (head-prel? rel wrd-inst)
		(let* ((oset (cog-outgoing-set rel))
				(head (car (cog-outgoing-set (cadr oset))))
			)
			(and 
				(equal? head wrd-inst)
				(eq? 'PrepositionalRelationshipNode (cog-type (car oset)))
			)
		)
	)

	; Get all relations, and filter them out.
	(filter! 
		(lambda (rel) (head-prel? rel word-inst))
		(word-inst-get-prep-relations word-inst)
	)
)

; ---------------------------------------------------------------------
; word-inst-get-subscript-str   Get link-grammar subscript for word-inst
; 
; Given a word instance, return the subscript string for it.
; The "subscript string" is the part that follows the period in
; the link-grammar dictionary. Thus, for the dictionary entry
; "red.a" the ".a" is the subscript.  Note that not all link-grammar
; dictionary entries will have a subscript.
;
(define (word-inst-get-subscript-str word-inst)
	; all subscripts start with a period as the first character.
	(define (subscript? tag)
		(if (char=? #\. (string-ref (cog-name tag) 0))
			#t #f
		)
	)
	(let ((infl (filter! subscript? (word-inst-get-attr word-inst))))
		(if (null? infl)
			""
			(cog-name (car infl))
		)
	)
)

; ---------------------------------------------------------------------
; word-inst-get-subscripted-word-str Get LG subscripted word string.
;
; Given a word instance, return the subscripted word.
; Here, "subscripted words" are link-grammar dictionary entries, for 
; example, "events.n" or "offered.v". Note that not all link-grammar
; dictionary entries will have a subscript.
;
(define (word-inst-get-subscripted-word-str word-inst)
	(string-append
		(word-inst-get-word-str word-inst)
		(word-inst-get-subscript-str word-inst)
	)
)

; ---------------------------------------------------------------------
; word-inst-get-senses   Get word senses associated with word.
;
; Given a word instance, return a list of the word-senses associated
; with the word.
(define (word-inst-get-senses word-inst)
	(cog-chase-link 'InheritanceLink 'WordSenseNode word-inst)
)

; ---------------------------------------------------------------------
; word-inst-sense-score  Get ranking score for word-inst & word-sense.
;
; Given a word-instance and a word-sense, return the numerical 
; word-sense ranking score. The rank is stored as the "count" part of the
; count truth value of the InheritanceLink connecting the two: i.e. in
; the structure
;
;     InheritanceLink (ctv 1.0 0.0 -0.006025)
;        WordInstanceNode "day@171506d8f3"
;        WordSenseNode "daylight%1:28:00::"
;
; If there is no such link, return -999 as the score.
;
(define (word-inst-sense-score word-inst word-sense)
	(let ((slink (cog-link 'InheritanceLink word-inst word-sense)))
		(if (null? slink)
			-999.0
			(cdr (assoc 'count (cog-tv->alist (cog-tv slink))))
		)
	)
) 

; --------------------------------------------------------------------
; relation-get-dependent Get dependent part of a relation.
;
; Given a relation, return the dependent item in the relation. 
; That is, given the relation  _rel(head-word, dependent-word)
; return the dependent-word.  This does no error checking.
;
(define (relation-get-dependent rel)
	; cadr because the dependent word is second in the relation.
	(cadr 
		(cog-outgoing-set
			; cadr because ListLink is second in the EvaluationLink
			(cadr
				(cog-outgoing-set rel)
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

	; Delete, recusively, all of the word-instances in the parse.
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

	; For each parse of the sentence, delete parse
	(for-each 
		(lambda (x) 
			(if (eq? 'ParseLink (cog-type x))
				(delete-parse (car (cog-outgoing-set x)))
			)
		)
		(cog-incoming-set sent)
	)

	; This delete will fail if there are still incoming links ... 
	; this is intentional. Its up to the caller to cleanup.
	(cog-delete sent)
)

; ---------------------------------------------------------------------
; delete-sentences       Delete all atoms that occur in sentences
;
; Delete atoms that belong to particular sentence instances; we don't
; want to clog up the atomspace with old junk we don't want any more.
;
; XXX TODO: In principle, this could be accomplished by lowering the
; AttentionValue for the atoms, and letting the ForgettingAgent do its
; work. In practice, this is too complicated, for now.
;
(define (delete-sentences)
	(let ((n 0))
	; (define (delit atom) (set! n (+ n 1)) #f)
	; (define (delit atom) (cog-delete-recursive atom) #f)
	(define (delit atom) (cog-delete-recursive atom) (set! n (+ n 1)) #f)

	; (define (delone atom) (cog-delete atom) #f)
	(define (delone atom) (cog-delete atom) (set! n (+ n 1)) #f)

	; Can't delete InheritanceLink, its used to mark wsd completed... 
	; (cog-map-type delone 'InheritanceLink)
	; Can't delete EvaluationLink, these are used elsewhere.
	; (cog-map-type delone 'EvaluationLink)
	; Can't delete ListLink, these are used in EvaluationLinks.
	; (cog-map-type delone 'ListLink)
	
	; Part of Speech links are used in the word-sense 
	; database, so cannot delete these.
	; (cog-map-type delone 'PartOfSpeechLink)
	; Can't delete LemmaLink's; used by the triples processing rules
	; (cog-map-type delone 'LemmaLink)

	(cog-map-type delone 'ParseLink)
	(cog-map-type delone 'ReferenceLink)

	(cog-map-type delone 'CosenseLink)

	; Its sort of a shame to delete the SimilarityLinks, but these
	; do make the server bloat after a while.
	(cog-map-type delone 'SimilarityLink)

	; We do delete all the test-processing nodes, recursively.
	; This should make any links that contain these to go "poof",
	; and so the above link deletions should not really be needed
	; (except for the similarity links).
	(cog-map-type delit 'DocumentNode)
	(cog-map-type delit 'SentenceNode)
	(cog-map-type delit 'ParseNode)
	(cog-map-type delit 'WordInstanceNode)

	; Pointless to delete these, since there should only be 
	; a few hundred of these, total.
	; (cog-map-type delit 'LinkGrammarRelationshipNode) 
	; (cog-map-type delit 'DefinedLinguisticConceptNode) 
	; (cog-map-type delit 'DefinedLinguisticRelationshipNode) 

(system (string-join (list "echo deleted: " (number->string n) )))
	)
)

; =============================================================
