;
; nlp-utils.scm
;
;;; Commentary:
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
; -- sentence-get-utterance-type  Get the speech act of a sentence.
; -- sent-list-get-parses   Get parses of a list of sentences.
; -- sent-get-words-in-order  Get all words occuring in a sentence in order.
; -- sent-get-interp Get all the InterpretationNodes of a sentence.
; -- parse-get-words        Get all words occuring in a parse.
; -- parse-get-words-in-order  Get all words occuring in a parse in order.
; -- parse-get-relations    Get all RelEx relations in a parse.
; -- parse-get-relex-outputs  Get all RelEx outputs in a parse.
; -- parse-get-r2l-outputs  Get all R2L outputs in a parse.
; -- interp-get-r2l-outputs Get all R2L outputs in an Interpretation.
; -- interp-get-parse       Get the Interpretation of a Parse.
; -- word-inst-get-parse    Return the ParseNode associated with word-inst.
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
; -- word-inst-get-disjunct Get the disjunct (LgAnd) used for a word-inst.
; -- relation-get-dependent Get dependent part of a relation.
; -- delete-sentence        Delete all atoms associated with a sentence.
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
; Copyright (c) 2015 OpenCog Foundation
;

(use-modules (ice-9 receive) (srfi srfi-1))

; ---------------------------------------------------------------------
(define-public (map-parses proc sent-or-list)
"
  map-parses   Call proceedure on every parse of the sentence.

  map-parses proc sent
  Call proceedure 'proc' on every parse of the sentence 'sent'

  Expected input is a SentenceNode, or possibly a list of SentenceNodes.
  Each SentenceNode serves as an anchor to all of the parses of a sentence.
  It is connected via ParseLink's to each of the individual parses of the
  sentence. This routine backtracks over the ParseNode to find these.

  The recursion will stop if proc returns something other than #f. This
  routine returns the last value that stopped the recursion. (In other
  words, this is not really a map, but something kind-of weird -- XXX
  this should probably be fixed -- TODO)
"
	(cog-map-chase-links-chk 'ParseLink 'ParseNode
		proc sent-or-list 'SentenceNode)
)

; Same as above, but multi-threaded -- each parse dispatched to its own
; thread, on a distinct CPU.
(define-public (parallel-map-parses proc sent-or-list)
	(cog-par-chase-links-chk 'ParseLink 'ParseNode
		proc sent-or-list 'SentenceNode)
)

; ---------------------------------------------------------------------
(define-public (map-word-instances proc parse-or-list)
"
  map-word-instances   Call proc on each word-instance of parse.

  map-word-instances proc parse
  Call proceedure 'proc' on each word-instance of 'parse'

  Expected input is a ParseNode or a list of ParseNodes. These serve
  as anchors to all of the word instances in a parse. The word instances
  can be found by back-tracking through the WordInstanceLink to the
  individual words, which is what this method does.
"
	(cog-map-chase-links-chk 'WordInstanceLink 'WordInstanceNode
		proc parse-or-list 'ParseNode)
)

; ---------------------------------------------------------------------
(define-public (map-word-node proc word-inst)
"
  map-word-node        Call proc on the word-node associated to word-inst.

  map-word-node proc word-inst
  Call proceedure 'proc' on the word-node associated to 'word-inst'

  Expected input is a WordInstanceNode, which serves as an anchor
  to a word instance. The WordInstanceNode is connnected via a ReferenceLink
  to the actual word node.
"
	(cog-map-chase-links-chk 'ReferenceLink 'WordNode
		proc word-inst 'WordInstanceNode)
)

; ---------------------------------------------------------------------
(define-public (document-get-sentences doco)
"
  document-get-sentences Get senteces in document.

  document-get-sentences doco

  Given a document, return a list of sentences in that document
  Throws an error if doco is not a DocumentNode
"
	(if (eq? (cog-type doco) 'DocumentNode)
		(cog-get-reference doco)
		(throw 'wrong-atom-type 'document-get-sentences
			"Error: expecting DocumentNode:" doco)
	)
)

; ---------------------------------------------------------------------
(define-public (sentence-get-parses sent-node)
"
  sentence-get-parses    Get parses of a sentence.

  sentence-get-parses sent-node

  Given a sentence, return a list of parses in that sentence
  Basically, chase a ParseLink to a ParseNode
  Throws an error if sent-node is not a SentenceNode
"
	(cog-chase-link-chk 'ParseLink 'ParseNode sent-node 'SentenceNode)
)

; -----------------------------------------------------------------------
(define-public (sent-list-get-parses sent-list)
"
  sent-list-get-parses   Get parses of a list of sentences.

  Given a list of sentences, return a list of parses of those sentences.
  That is, given a List of SentenceNode's, return a list of ParseNode's
  associated with those sentences.
"
	(concatenate! (map sentence-get-parses sent-list))
)

;------------------------------------------------------------------
(define-public (sentence-get-utterance-type sent)
"
  sentence-get-utterance-type SENT -- Check the utterance speech act type

  Expect SENT to be (SentenceNode \"sentence@45c470a6-29...\")
  Will return (DefinedLinguisticConceptNode ACT) where ACT is
  one of DeclarativeSpeechAct, InterrogativeSpeechAct,
  TruthQuerySpeechAct, etc...
"
	; XXX TODO (1) this could be converted into a simple GetLink
	; and probably should be. (2) There should be a syntax for GetLink
	; that is lest verbose, and closer in style to what is written
	; below. Viz, it should be posible to write GetLink's as a sequence
	; of chases.

	; parse will be (ParseNode "sentence@a6_parse_0")
	(define parse (car (cog-chase-link 'ParseLink 'ParseNode sent)))

	; interp will be (InterpretationNode "sentence@a610_interpretation_$X")
	(define interp (car
		(cog-chase-link 'InterpretationLink 'InterpretationNode parse)))

	; act-type will be (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
	(define act-type (cog-chase-link
		'InheritanceLink 'DefinedLinguisticConceptNode interp))

	; Return act-type
	act-type
)

; ---------------------------------------------------------------------
(define-public (sent-get-words-in-order sent-node)
"
  sent-get-words-in-order - Given a sentence, return a list of all words in order

  Given a sentence, return all word instances in order
"
	(map parse-get-words-in-order (sentence-get-parses sent-node))
)

; ---------------------------------------------------------------------
(define-public (sent-get-interp sent-node)
"
  sent-get-interp - Given a SentenceNode returns a list of InterpretationNodes
"
    (parse-get-interp (car (sentence-get-parses sent-node)))
)

; ---------------------------------------------------------------------
(define-public (parse-get-words parse-node)
"
  parse-get-words - Given a parse, return a list of all words in the parse

  Given a parse, return all word instances in arbitary order
  This version is faster than the in order version.
"
	(cog-chase-link 'WordInstanceLink 'WordInstanceNode parse-node)
)

; ---------------------------------------------------------------------
(define-public (parse-get-words-in-order parse-node)
"
  parse-get-words-in-order - Given a parse, return a list of all words in the parse in order

  Given a parse, return all word instances in order
"
	(define word-inst-list (cog-chase-link 'WordInstanceLink 'WordInstanceNode parse-node))
	(define number-list (map word-inst-get-number word-inst-list))
	(define (less-than word-inst-1 word-inst-2)
		(define index-1 (list-index (lambda (a-node) (equal? word-inst-1 a-node)) word-inst-list))
		(define index-2 (list-index (lambda (a-node) (equal? word-inst-2 a-node)) word-inst-list))
		(< (string->number (cog-name (list-ref number-list index-1))) (string->number (cog-name (list-ref number-list index-2)))))
	(sort word-inst-list less-than)
)

; --------------------------------------------------------------------
(define-public (parse-get-relations parse-node)
"
  parse-get-relations    Get all RelEx relations in a parse.

  Given a parse, return a list of all relex relations
"
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

; --------------------------------------------------------------------
(define-public (parse-get-relex-outputs parse-node)
"
  parse-get-relex-outputs  Get all RelEx outputs in a parse.

  Given a parse, returns a list of RelEx outputs associated with
  the ParseNode.
"
    (define parse-link (cog-incoming-by-type parse-node 'ParseLink))
    (define sent-node (gdr (car parse-link)))
    (define sent-seq-link (cog-incoming-by-type sent-node 'SentenceSequenceLink))

    (define word-inst-links (cog-incoming-by-type parse-node 'WordInstanceLink))
    (define word-inst-nodes (map gar word-inst-links))
    (define word-seq-link (append-map (lambda (n)
        (cog-incoming-by-type n 'WordSequenceLink)) word-inst-nodes))
    (define pos-link (append-map (lambda (n)
        (cog-incoming-by-type n 'PartOfSpeechLink)) word-inst-nodes))
    (define lemma-link (append-map (lambda (n)
        (cog-incoming-by-type n 'LemmaLink)) word-inst-nodes))
    (define tense-link (append-map (lambda (n)
        (cog-incoming-by-type n 'TenseLink)) word-inst-nodes))
    (define inherit-link (append-map (lambda (n)
        (cog-incoming-by-type n 'InheritanceLink)) word-inst-nodes))
    (define ref-win (filter (lambda (l)
        (equal? (cog-type (gdr l)) 'WordNode)) (append-map
            (lambda (n) (cog-incoming-by-type n 'ReferenceLink)) word-inst-nodes)))

    (define eval-dlrn (append-map (lambda (n)
        (cog-get-pred n 'DefinedLinguisticRelationshipNode)) word-inst-nodes))
    (define eval-lgrn (append-map (lambda (n)
        (cog-get-pred n 'LinkGrammarRelationshipNode)) word-inst-nodes))
    (define eval-lgin (append-map (lambda (n)
        (cog-get-pred n 'LgLinkInstanceNode)) word-inst-nodes))

    (define lg-inst-nodes (map gar eval-lgin))
    (define lg-word-cset (append-map (lambda (n)
        (cog-incoming-by-type n 'LgWordCset)) word-inst-nodes))
    (define ref-lgin (append-map (lambda (n)
        (cog-incoming-by-type n 'ReferenceLink)) lg-inst-nodes))
    (define lg-link-inst-link (append-map (lambda (n)
        (cog-incoming-by-type n 'LgLinkInstanceLink)) lg-inst-nodes))

    (delete-duplicates (append
        parse-link
        sent-seq-link
        word-inst-links
        word-seq-link
        pos-link
        lemma-link
        tense-link
        inherit-link
        ref-win
        eval-dlrn
        eval-lgrn
        eval-lgin
        ref-lgin
        lg-word-cset
        lg-link-inst-link
    ))
)

; --------------------------------------------------------------------
(define-public (parse-get-r2l-outputs parse-node)
"
  parse-get-r2l-outputs    Get all R2L outputs in a parse.

  Given a parse, returns a list of the r2l logic outputs associated
  with the ParseNode.
"
	(let ((inters (cog-chase-link 'InterpretationLink 'InterpretationNode parse-node)))
		(if (null? inters)
			'()
			(interp-get-r2l-outputs (car inters))
		)
	)
)

; --------------------------------------------------------------------
(define-public (parse-get-interp parse-node)
"
  parse-get-interp    Get the interpretations of the parse.

  Returns the InterpretationNodes associated with a ParseNode.
"
    (cog-chase-link 'InterpretationLink 'InterpretationNode parse-node)
)

; --------------------------------------------------------------------
(define-public (interp-get-r2l-outputs interp-node)
"
  interp-get-r2l-outputs    Get all R2L outputs in an Interpretation.

  Given an interpretation, returns a list of the r2l logic outputs
  associated with the InterpretationNode.
"
	(cog-outgoing-set (list-ref (cog-chase-link 'ReferenceLink 'SetLink interp-node) 0))
)

; --------------------------------------------------------------------
(define-public (interp-get-parse interp)
"
  interp-get-parse    Get the parse that resulted in the given interpretation.

  Returns the ParseNode associated with an InterpretationNode.
"
	(car (cog-chase-link 'InterpretationLink 'ParseNode interp))
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-parse word-inst)
"
  word-inst-get-parse   Return the ParseNode associated with word-inst

  Return the ParseNode associated with 'word-inst'
"
	(car (cog-chase-link 'WordInstanceLink 'ParseNode word-inst))
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-number word-inst)
"
  word-inst-get-number   Return the NumberNode associated with word-inst

  Return the NumberNode associated with 'word-inst'
"
	(car (cog-chase-link 'WordSequenceLink 'NumberNode word-inst))
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-word word-inst)
"
  word-inst-get-word   Return the WordNode associated with word-inst

  Return the WordNode associated with 'word-inst'
"
	(cog-chase-link 'ReferenceLink 'WordNode word-inst)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-word-str word-inst)
"
  word-inst-get-word-str  Return the word string assoc with word-inst

  Return the word string associated with the word-instance
"
	(cog-name (car (word-inst-get-word word-inst)))
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-lemma word-inst)
"
  word-inst-get-lemma  Return the lemma of word instance.

  Given a word instance, return the lemma for of the word.
  Also works if the word-inst is actually a seme.
"
	(let ((wlist (cog-chase-link 'LemmaLink 'WordNode word-inst)))
		(if (null? wlist)
			; FIXME: this is a dumb way to get other type
			(let ((nlist (cog-chase-link 'LemmaLink 'NumberNode word-inst)))
				(if (null? nlist)
					'()
					(car nlist)
				)
			)
			(car wlist)
		)
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-attr word-inst)
"
  word-inst-get-attr   Return attributes of word instance

  Given a word instance, return a list of attributes for the word.
"
	(cog-chase-link 'InheritanceLink 'DefinedLinguisticConceptNode
		word-inst
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-pos word-inst)
"
  word-inst-get-pos    Return part-of-speech (POS) of word instance

  Return the part-of-speech of a word-instance
"
	(cog-chase-link 'PartOfSpeechLink 'DefinedLinguisticConceptNode
		word-inst
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-match-pos? word-inst pos)
"
  word-inst-match-pos? Does word-inst have POS?

  Return #t is the word-instance has part-of-speech pos else #f
  'word-inst' must have a PartOfSpeechLink on it; typically its
  a WordInstanceNode.
  'pos' is a string, typically 'noun' or 'verb'.

  Not all word instances are always tagged. For example, in a sentence
  with 'John Kennedy', John won't have a pos tag, because RelEx will
  have already meged this into an entity string (and tagged the string).
"
	(let ((pos-list (word-inst-get-pos word-inst)))
		(if (null? pos-list)
			#f
			(string=? pos (cog-name (car pos-list)))
		)
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-is-noun? word-inst)
"
  word-inst-is-noun?   Is word instance a noun?

  Return #t is the word-instance is a noun
"
	(word-inst-match-pos? word-inst "noun")
)

; ---------------------------------------------------------------------
(define-public (word-inst-is-verb? word-inst)
"
  word-inst-is-verb?   Is word instance a verb?

  Return #t is the word-instance is a verb
"
	(word-inst-match-pos? word-inst "verb")
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-relations word-inst)
"
  word-inst-get-relations       Get RelEx relations involving word-inst

  Given a word instance, return a list of relations the word participates in.
  That is, given (WordInstanceNode 'dog'), return all _subj(*,dog)
  _subj(dog,*), _obj(dog,*) etc.

  See also word-inst-get-head-relations and
  word-inst-get-relex-modifiers below.
"
	(cog-get-pred word-inst 'DefinedLinguisticRelationshipNode)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-prep-relations word-inst)
"
  word-inst-get-prep-relations  Get prepositional relations for word-inst
"
	(cog-get-pred word-inst 'PrepositionalRelationshipNode)
)

; --------------------------------------------------------------------
(define-public (word-inst-get-head-relations word-inst)
"
  word-inst-get-head-relations  Get relations with word-inst as head

  Given a word-instance, return a list of relex relations that
  this word-instance is the head-word of. (It is the head-word
  if it is first e.g. _amod(head-word, attr-word)
"
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
(define-public (word-inst-filter-relex-rels word-inst rel-name-list)
"
  word-inst-filter-relex-rels   Get filtered set of RelEx relations

  Given a word-instance and a list of relex relation names, return a
  list of the relex relations for which this word-instance is the
  head-word.  The head-word is the first word in the relation --
  for example _amod(head-word, dependent-word)
"
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
(define-public (verb-inst-get-relex-rels word-inst)
"
  verb-inst-get-relex-rels      Get relations for a verb.

  Given a verb word-instance, return a list of relex relations that
  this word-instance is the head-word of. (It is the head-word
  if it is first e.g. _subj(head-word, dependency-word)
"
	; There are a few other relations we should probably deal with,
	; e.g. _predadj Right now, this is unclear.
	; Modifiers are documented at:
	; http://opencog.org/wiki/Binary_relations
	(word-inst-filter-relex-rels word-inst
		(list "_advmod" "_iobj" "_obj" "_subj")
	)
)

; --------------------------------------------------------------------
(define-public (noun-inst-get-relex-modifiers word-inst)
"
  noun-inst-get-relex-modifiers Get relations for a noun.

  Given a noun word-instance, return a list of relex modifiers that
  this word-instance is the head-word of. (It is the head-word
  if it is first e.g. _amod(head-word, attr-word)

  This very explicitly returns *only* modifiers, and not relations in
  general.
"
	; There are a few other modifiers we should probably deal with,
	; including quantity multiplier, etc. Right now, this is unclear.
	; Modifiers are documented at:
	; http://opencog.org/wiki/Binary_relations
	(word-inst-filter-relex-rels word-inst
		(list "_amod" "_appo" "_nn" "_poss" "_%quantity")
	)
)

; --------------------------------------------------------------------
(define-public (noun-inst-get-prep-rels word-inst)
"
  noun-inst-get-prep-rels       Get prep relations for noun.

  Given a noun word-instance, return a list of prepositional relations
  that this word-instance is the head-word of. (It is the head-word
  if it is first e.g. prep(head-word, dependent-word)
"
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
(define-public (word-inst-get-subscript-str word-inst)
"
  word-inst-get-subscript-str   Get link-grammar subscript for word-inst

  Given a word instance, return the subscript string for it.
  The 'subscript string' is the part that follows the period in
  the link-grammar dictionary. Thus, for the dictionary entry
  'red.a' the '.a' is the subscript.  Note that not all link-grammar
  dictionary entries will have a subscript.
"
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
(define-public (word-inst-get-subscripted-word-str word-inst)
"
  word-inst-get-subscripted-word-str Get LG subscripted word string.

  Given a word instance, return the subscripted word.
  Here, 'subscripted words' are link-grammar dictionary entries, for
  example, 'events.n' or 'offered.v'. Note that not all link-grammar
  dictionary entries will have a subscript.
"
	(string-append
		(word-inst-get-word-str word-inst)
		(word-inst-get-subscript-str word-inst)
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-senses word-inst)
"
  word-inst-get-senses   Get word senses associated with word.

  Given a word instance, return a list of the word-senses associated
  with the word.
"
	(cog-chase-link 'InheritanceLink 'WordSenseNode word-inst)
)

; ---------------------------------------------------------------------
(define-public (word-inst-sense-score word-inst word-sense)
"
  word-inst-sense-score  Get ranking score for word-inst & word-sense.

  Given a word-instance and a word-sense, return the numerical
  word-sense ranking score. The rank is stored as the 'count' part of the
  count truth value of the InheritanceLink connecting the two: i.e. in
  the structure

      InheritanceLink (ctv 1.0 0.0 -0.006025)
         WordInstanceNode 'day@171506d8f3'
         WordSenseNode 'daylight%1:28:00::'

  If there is no such link, return -999 as the score.
"
	(let ((slink (cog-link 'InheritanceLink word-inst word-sense)))
		(if (null? slink)
			-999.0
			(cdr (assoc 'count (cog-tv->alist (cog-tv slink))))
		)
	)
)

; ---------------------------------------------------------------------
(define-public (word-inst-get-disjunct word-inst)
"
  word-inst-get-disjunct -- Get the disjunct (LgAnd) used for a word-inst.
"
	(car (cog-chase-link 'LgWordCset 'LgAnd word-inst))
)

; --------------------------------------------------------------------
(define-public (relation-get-dependent rel)
"
  relation-get-dependent Get dependent part of a relation.

  Given a relation, return the dependent item in the relation.
  That is, given the relation  _rel(head-word, dependent-word)
  return the dependent-word.  This does no error checking.
"
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
(define-public (delete-sentence sent)
"
  delete-sentence -- delete all atoms associated with a sentence.

  Delete the parses and word-instances associated with the sentence,
  including LemmaLink's, ReferenceLinks, RelEx relations, and
  link-grammar linkages.

  Only the atoms in the atomspace are removed; if any are in the
  backingstore (database), they are untouched.

  Expect SENT to be a SentenceNode; thus the general structure is
  expected to be:

    ParseLink
         ParseNode
         SentenceNode
"
	; Purge stuff associated with a single LgLinkInstanceNode
	(define (extract-link-instance li)
		(if (not (null? li)) (cog-extract-recursive li))
	)

	; Purge stuff associated with a single word-instance.
	; Expects wi to be a WordInstanceNode.
	; Calling extract-recursive will blow away most of the junk
	; that the WordInstance appear in, but a few have to be removed
	; manually. These are:
	;
	; The WordInstances that appear in LgLinkInstances:
	;     EvaluationLink
	;           LgLinkInstanceNode
	;           ListLink
	;               WordInstanceNode
	;               WordInstanceNode
	;
	; and so we have to track those down and extract them.
	;
	; They also appear in WordSequenceLinks:
	;     WordSequenceLink
	;         WordInstanceNode
	;         NumberNode
	; and so we need to get rid of the NumberNode's too.
	;
	; The extract-recursive will blow away everything else --
	; the LemmaLinks, ReferenceLinks, etc.

	(define (extract-word-instance wi)
		(for-each
			(lambda (x)
				(if (eq? 'ListLink (cog-type x))
					(for-each extract-link-instance
						(cog-chase-link 'EvaluationLink 'LgLinkInstanceNode x))
				)
				; Extract the NumberNode
				(if (eq? 'WordSequenceLink (cog-type x))
					(let ((oset (cog-outgoing-set x)))
						(cog-extract x)
						(cog-extract (cadr oset)))
				)
			)
			(cog-incoming-set wi)
		)
		(cog-extract-recursive wi)
	)

	; Purge, recusively, all of the word-instances in the parse.
	; This is expecting 'parse' to be a ParseNode.
	; The following is expected:
	;      WordInstanceLink
	;           WordInstanceNode
	;           ParseNode
	;
	(define (extract-parse parse)
		(for-each
			(lambda (x)
				(if (eq? 'WordInstanceLink (cog-type x))
					(extract-word-instance (car (cog-outgoing-set x)))
				)
			)
			(cog-incoming-set parse)
		)
		(cog-extract-recursive parse)
	)

	; For each parse of the sentence, extract the parse
	; This is expecting a structure
	;     ParseLink
	;         ParseNode     car of the outgoing set
	;         SentenceNode
	(for-each
		(lambda (x)
			(if (eq? 'ParseLink (cog-type x))
				; The car will be a ParseNode
				(extract-parse (car (cog-outgoing-set x)))
			)
			; Extract the NumberNode
			(if (eq? 'SentenceSequenceLink (cog-type x))
				(let ((oset (cog-outgoing-set x)))
					(cog-extract x)
					(cog-extract (cadr oset)))
			)
		)
		(cog-incoming-set sent)
	)

	; This delete will fail if there are still incoming links ...
	; this is intentional. Its up to the caller to cleanup.
	(cog-extract sent)
)

; ---------------------------------------------------------------------
(define-public (delete-sentences)
"
  delete-sentences       Delete all atoms that occur in sentences

  Delete atoms that belong to particular sentence instances; we don't
  want to clog up the atomspace with old junk we don't want any more.

  Only the atoms in the atomspace are removed; if any are in the
  backingstore (database), they are untouched.

  XXX TODO: In principle, this could be accomplished by lowering the
  AttentionValue for the atoms, and letting the ForgettingAgent do its
  work. In practice, this is too complicated, for now.

  XXX: Some of these nodes & links are needed by the language generation
  pipeline for matching to old sentences, so maybe they cannot be
  removed.
"
	(let ((n 0))
	; (define (delit atom) (set! n (+ n 1)) #f)
	; (define (delit atom) (cog-extract-recursive atom) #f)
	(define (delit atom) (cog-extract-recursive atom) (set! n (+ n 1)) #f)

	; (define (delone atom) (cog-extract atom) #f)
	; (define (delone atom) (cog-extract atom) (set! n (+ n 1)) #f)
	(define (delone atom) (extract-hypergraph atom) (set! n (+ n 1)) #f)

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
	(cog-map-type delone 'LgLinkInstanceLink)

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
	(cog-map-type delit 'LgLinkInstanceNode)

	; Pointless to delete these, since there should only be
	; a few hundred of these, total.
	; (cog-map-type delit 'LinkGrammarRelationshipNode)
	; (cog-map-type delit 'DefinedLinguisticConceptNode)
	; (cog-map-type delit 'DefinedLinguisticRelationshipNode)

(system (string-join (list "echo deleted: " (number->string n) )))
	)
)

; ---------------------------------------------------------------------
(define-public (use-relex-server HOSTNAME PORTNUM)
"
  use-relex-server HOSTNAME PORTNUM

  Use the RelEx server found at HOSTNAME:PORTNUM for parsing.
  Any sentences to be parsed will be sent to this server.

  HOSTNAME can be either a TCPIPv4 hostname or IP address, for example,
  \"foo.bar.org\" or \"localhost\" or \"127.0.0.1\".

  PORTNUM must be a numeric port number in the range 1-65535

  The IP address is returned.  If the HOSTNAME is invalid, the
  currently-set server is not changed, and an exception is thrown.
"
	(define hosty (gethost HOSTNAME))
	(set! relex-server-host
		(inet-ntop (hostent:addrtype hosty)
			(car (hostent:addr-list hosty))))

	; Basic sanity check.
	(if (not (exact-integer? PORTNUM)) (throw 'bad-portnum))
	(set! relex-server-port PORTNUM)
	relex-server-host
)

(define-public (set-relex-server-host)
"
  Sets the relex-server address. To be used only in a docker setup.
"
	(catch
		#t
		(lambda () (use-relex-server "relex" 4444))
		(lambda (key . rest) relex-server-host))
)

; =============================================================
