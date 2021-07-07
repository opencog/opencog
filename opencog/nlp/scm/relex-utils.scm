;
; relex-utils.scm
;
;;; Commentary:
;
; Assorted RelEx-related utilities.  Note that RelEx is partly
; deprecated; it only works for English, and is unlikely to be
; developed further in the future.
;
; Operations include:
; -- looping over all RelEx relations
; -- get part-of-speech, lemma of word.
; -- get prepositions
;
; The function names that can be found here are:
; -- parse-get-relex-relations    Get all RelEx relations in a parse.
; -- parse-get-relex-outputs  Get all RelEx outputs in a parse.
; -- parse-get-r2l-outputs  Get all R2L outputs in a parse.
; -- interp-get-r2l-outputs Get all R2L outputs in an Interpretation.
; -- word-inst-get-lemma  Return the lemma of word instance.
; -- word-inst-get-attr   Return attributes of word instance.
; -- word-inst-get-pos    Return part-of-speech (POS) of word instance.
; -- word-inst-match-pos? Does word-inst have POS?
; -- word-inst-is-noun?   Is word instance a noun?
; -- word-inst-is-verb?   Is word instance a verb?
; -- word-inst-get-relex-relations Get RelEx relations involving word-inst.
; -- word-inst-get-head-relations  Get relations with word-inst as head.
; -- word-inst-get-prep-relations  Get prepositional relations for word-inst.
; -- word-inst-filter-relex-rels   Get filtered set of RelEx relations.
; -- verb-inst-get-relex-rels      Get relations for a verb.
; -- noun-inst-get-relex-modifiers Get relations for a noun.
; -- noun-inst-get-prep-rels       Get prep relations for noun.
; -- word-inst-get-subscript-str   Get link-grammar subscript for word-inst.
; -- word-inst-get-subscripted-word-str Get LG subscripted word string.
; -- relation-get-dependent Get dependent part of a relation.


; Copyright (c) 2008, 2009, 2013 Linas Vepstas <linasvepstas@gmail.com>
;

(use-modules (ice-9 regex))
(use-modules (srfi srfi-1))
(use-modules (ice-9 textual-ports)) ; For get-string-all

; --------------------------------------------------------------------
(define-public (parse-get-relex-relations parse-node)
"
  parse-get-relex-relations    Get all RelEx relations in a parse.

  Given a parse, return a list of all relex relations
"
	; Get a list of words in the parse
	; Get a list of lists of relations for each word
	; concatenate! will reduce the list of lists to a plain list
	; Remove duplicates
	(delete-duplicates!
		(concatenate!
			(map word-inst-get-relex-relations
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
        (cog-get-pred n 'LgLinkNode)) word-inst-nodes))
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
(define-public (sent-get-r2l-outputs sent-node)
"
  sent-get-r2l-outputs SENT-NODE

  Returns a list of the r2l logic outputs associated with SENT-NODE, assuming
  there is only one interpretation.
"

  (interp-get-r2l-outputs (car (sent-get-interp sent-node)))
)

; --------------------------------------------------------------------
(define-public (parse-get-r2l-outputs parse-node)
"
  parse-get-r2l-outputs    Get all R2L outputs in a parse.

  Given a parse, returns a list of the r2l logic outputs associated
  with the ParseNode.
"
	(let ((inters (cog-chase-link 'InterpretationLink 'InterpretationNode parse-node)))
		(if (nil? inters)
			'()
			(interp-get-r2l-outputs (car inters))
		)
	)
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

; ---------------------------------------------------------------------
(define-public (word-inst-get-lemma word-inst)
"
  word-inst-get-lemma  Return the lemma of word instance.

  Given a word instance, return the lemma for of the word.
  Also works if the word-inst is actually a seme.
"
	(let ((wlist (cog-chase-link 'LemmaLink 'WordNode word-inst)))
		(if (nil? wlist)
			; FIXME: this is a dumb way to get other type
			(let ((nlist (cog-chase-link 'LemmaLink 'NumberNode word-inst)))
				(if (nil? nlist)
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
		(if (nil? pos-list)
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
(define-public (word-inst-get-relex-relations word-inst)
"
  word-inst-get-relex-relations       Get RelEx relations involving word-inst

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
		(word-inst-get-relex-relations word-inst)
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
		(word-inst-get-relex-relations word-inst)
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
		(if (nil? infl)
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
		(cog-name (word-inst-get-word word-inst))
		(word-inst-get-subscript-str word-inst)
	)
)

; ---------------------------------------------------------------------
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

; ---------------------------------------------------------------------
; User-modifiable config parameters.
; We'll keep these here for backwards-compat, for now, but it is
; recommended that (use-relex-server HOST PORT) be used instead...
(define-public relex-server-host "127.0.0.1")
(define-public relex-server-port 4444)

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

(define-public (relex-reachable?)
"
  relex-reachable?

  Returns #t if the relex server is reachable, #f either wise. This can be
  used as a circuit breaker while writing unit or integration tests.
"
  (define result "")
  (define s (socket PF_INET SOCK_STREAM 0))
  (define connected
    (catch #t
      (lambda () (connect s AF_INET (inet-pton AF_INET relex-server-host)
        relex-server-port))
      (lambda (key . args) #f)))

  (if connected
    (begin
      ; An explicit port-encoding is needed by guile-2.0.9. Doesn't hurt for
      ; higher versions as well.
      (set-port-encoding! s "utf-8")
      ; Send whitespace so as to not pollute atomspace while checking
      (display " \n" s)
      (set! result (get-string-all s))
      (close-port s)
      (if (string-match "; NO PARSES\n" result) #t #f)
    )
    #f
  )
)

; =============================================================
