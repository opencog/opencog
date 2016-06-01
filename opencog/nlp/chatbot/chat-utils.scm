;
; chat-utils.scm
;
; Chatbot dialog utils.
; These include performing R2L processing and question-answering.
;
; Copyright (c) 2009 Linas Vepstas <linasvepstas@gmail.com>
; Copyright (c) 2015 OpenCog Foundation
;

(use-modules (ice-9 threads)  ; needed for par-map
             (srfi srfi-1)
             (opencog)
             (opencog atom-types)
             (opencog rule-engine)
             (opencog nlp)
             (opencog nlp fuzzy)
             (opencog nlp microplanning)
             (opencog nlp relex2logic))

; -----------------------------------------------------------------------
(define (r2l-parse sent)
"
  r2l-parse SENT -- perform relex2logic processing on sentence SENT.

  Runs the rules found in R2L-en-RuleBase over the RelEx output
  creating the logical representation of sentence in the atomspace.
  Returns a list containing SetLinks that are the r2l-interpretations
  for individual parses.

  This can't handle  mutliple thread execution (Why???). Thus, mapping
  this function over a list of sentences, even though possible, is not
  advised.

  SENT must be a SentenceNode.
"
    (define (cog-extract-parent a-link is-from-fc)
        ; Many rules return a ListLink of results that they
        ; generated. Some rules return singletons. And the
        ; Forward Chainer uses a SetLink to wrap all these
        ; results. So if A-LINK is a ListLink or is directly
        ; from the FC, then delete it and return a list of
        ; its contents, else return a list holding A-LINK.
        ;
        ; XXX maybe this should be part of the ure module??
        (if (or (equal? 'ListLink (cog-type a-link)) is-from-fc)
            (let ((returned-list (cog-outgoing-set a-link)))
                    (cog-extract a-link)
                    returned-list)
            (list a-link))
    )

    (define (run-fc parse-node interp-link)
        ; This runs all the rules of R2L-en-RuleBase over relex parse
        ; outputs, and returns a cleaned and de-duplicated list. The
        ; relex outputs associated with 'parse-node' make the focus-set.
        ; This is done so that, IF there are multiple parses, then
        ; each is handled independently by passing it seperately, as
        ; each is likely to exist in a seperate semantic-universe.
        (define focus-set
            (SetLink (parse-get-relex-outputs parse-node) interp-link))
        (define outputs
            (cog-extract-parent (cog-fc (SetLink) r2l-rules focus-set) #t))

        (append-map (lambda (o) (cog-extract-parent o #f)) outputs)
    )

    (define (interpret parse-node)
        ; FIXME: Presently only a single interpretation is created for
        ; each parse. Multiple interpreation should be handled, when
        ; word-sense-disambiguation, anaphora-resolution and other
        ; post-processing are added to the pipeline.
        (let* ((interp-name (string-append(cog-name parse-node) "_interpretation_$X"))
               (interp-node (InterpretationNode interp-name))
               ; Associate the interpretation with a parse, as there
               ; could be multiplie interpretations for the same parse.
               (interp-link (InterpretationLink interp-node parse-node))
               (pre-result
                   (remove
                       (lambda (a) (equal? (cog-type a) 'ReferenceLink))
                       (delete-duplicates (run-fc parse-node interp-link))))
               (result (SetLink pre-result)))

            ; Construct a ReferenceLink to the output
            (ReferenceLink interp-node result)

            ; Time stamp the parse
            (AtTimeLink
                ; FIXME: maybe opencog's internal time octime should
                ; be used. Will do for now, assuming a single instance
                ; deals with a single conversation.
                (TimeNode (number->string (current-time)))
                interp-node
                (TimeDomainNode "Dialogue-System"))

            result
        )
    )

    (map interpret (sentence-get-parses sent))
)

; -----------------------------------------------------------------------
(define (r2l-count sent-list)
"
  r2l-count SENT -- maintain counts of R2L statistics for SENT-LIST.
"
	(define (update-tv nodes)
		(let ((default-stv (stv 1 0))
		      (default-k 800))
			(par-map
				(lambda (n)
					(if (equal? (cog-tv n) default-stv)
						(let* ((new-mean (/ 1 (cog-count-atoms (cog-type n))))
								(new-conf (/ 1 (+ 1 default-k))))
							(cog-set-tv! n (cog-new-stv new-mean new-conf))
						)
						(let* ((current-count (round (assoc-ref (cog-tv->alist (cog-tv n)) 'count)))
								(new-count (+ current-count 1))
								(new-mean (/ new-count (cog-count-atoms (cog-type n))))
								(new-conf (/ new-count (+ new-count default-k))))
							(cog-set-tv! n (cog-new-stv new-mean new-conf))
						)
					)
				)
				nodes
			)
		)
	)

	; Increment the R2L's node count value
	(parallel-map-parses
		(lambda (p)
			; The preferred algorithm is
			; (1) get all non-abstract nodes
			; (2) delete duplicates
			; (3) get the corresponding abstract nodes
			; (4) update count
			(let* ((all-nodes (append-map cog-get-all-nodes (parse-get-r2l-outputs p)))
			       ; XXX FIXME this is undercounting since each abstract node can have
			       ; multiple instances in a sentence.  Since there is no clean way
			       ; to get to the abstracted node from an instanced node yet, such
			       ; repeatition are ignored for now
			       (abst-nodes (delete-duplicates (filter is-r2l-abstract? all-nodes)))
			       (word-nodes (append-map word-inst-get-word (parse-get-words p))))
				(update-tv abst-nodes)
				(update-tv word-nodes)
			)
		)
		sent-list
	)
)

; -----------------------------------------------------------------------
(define-public (nlp-parse plain-text)
"
  nlp-parse PLAIN-TEXT -- Wrap most of the NLP pipeline in one function.

  Call the necessary functions for the full NLP pipeline.
"
	; Discard previous sentences, if any.
	(release-new-parsed-sents)

	; Check input to ensure that not-empty string isn't passed.
	(if (string=? plain-text "")
		(error "Please enter a valid sentence, not empty string"))

	; Call the RelEx server
	(relex-parse plain-text)

	(let ((sent-list (get-new-parsed-sentences)))
		; Unhook the anchor. MUST do this before r2l-parse, as
		; otherwise, parse-get-relex-outputs will wrap it in a
		; SetLink! Ouch!!
		(release-new-parsed-sents)

		; Perform the R2L processing.
		(r2l-parse (car sent-list))

		; Track some counts needed by R2L.
		(r2l-count sent-list)

		; Return the sentence list.
		sent-list
	)
)

; -----------------------------------------------------------------------
