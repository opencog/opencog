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

(use-modules (opencog logger))

; -----------------------------------------------------------------------

(use-modules (opencog) (opencog python) (opencog exec))

(python-eval "
from opencog.atomspace import AtomSpace, types, TruthValue

have_sentiment_analysis = True
try:
      import basic_sentiment_analysis
except ImportError:
      have_sentiment_analysis = False

atomspace = ''

def set_atomspace(atsp):
      global atomspace
      atomspace = atsp
      return TruthValue(1, 1)

def call_sentiment_parse(text_node, sent_node):
      global atomspace
      global have_sentiment_analysis

      if not have_sentiment_analysis:
          return TruthValue(1, 1)

      sentiment_score = basic_sentiment_analysis.sentiment_parse(text_node.name)
      if sentiment_score > 0:
          positive_node = atomspace.add_node(types.ConceptNode, 'Positive')
          atomspace.add_link(types.InheritanceLink, [sent_node, positive_node])
      elif sentiment_score < 0:
          negative_node = atomspace.add_node(types.ConceptNode, 'Negative')
          atomspace.add_link(types.InheritanceLink, [sent_node, negative_node])
      else:
          neutral_node = atomspace.add_node(types.ConceptNode, 'Neutral')
          atomspace.add_link(types.InheritanceLink, [sent_node, neutral_node])

      return TruthValue(1, 1)
")

; -----------------------------------------------------------------------
; TODO: Replace these time related utilities with one from TimeMap, when it is
; ready.
(define time-domain (TimeDomainNode "Dialogue-System"))

(define (sent-set-time sent)
"
  Associate time to the last sentence
"
    (AtTimeLink
        ; FIXME: maybe opencog's internal time octime should
        ; be used. Will do for now, assuming a single instance
        ; deals with a single conversation.
        (TimeNode (number->string (current-time)))
        sent
        time-domain)
)

(define-public (get-last-said-sent)
"
  Returns the SentenceNode of the last said sentence or returns an empty list.
"
    (define query
        (Get
            (VariableList
                (TypedVariableLink
                    (Variable "tn")
                    (TypeNode "TimeNode"))
                (TypedVariableLink
                    (Variable "s")
                    (TypeNode "SentenceNode")))
            (AtTimeLink
                (Variable "tn")
                (Variable "s")
                time-domain)))

    (define last-time 0)
    (define result '())
    (define (last-sent sent)
        (let ((sent-time (string->number (cog-name (gar sent)))))
            (if (>= sent-time last-time)
                (begin
                    (set! last-time sent-time)
                    (set! result (gdr sent)))
            )
        ))

    (let ((sents (cog-execute! query)))
        (for-each last-sent (cog-outgoing-set sents))
        (cog-delete sents)
        result
    )
)

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
            (sent-set-time sent)

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
        ; DEFAULT_TV and DEFAULT_K as defined in TruthValue.cc
        (let ((default-stv (stv 1 0))
              (default-k 800))
            (par-map
                (lambda (n)
                    (if (equal? (cog-tv n) default-stv)
                        (let ((new-mean (/ 1 (cog-count-atoms (cog-type n))))
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
; Control variable used to switch stimulation of WordNodes and
; WordInstanceNodes on parsing. This shouldn't be public.
(define nlp-stimulate-parses #f)

; The sti value that WordNodes and WordInstanceNodes are stimulated with.
(define nlp-stimulation-value 0)

; -----------------------------------------------------------------------
(define-public (nlp-start-stimulation STIMULUS)
"
  Switchs on the stimulation of WordNodes and WordInstanceNodes during parse
  by STI amount.
"
    (set! nlp-stimulation-value STIMULUS)
    (set! nlp-stimulate-parses #t)
)

; -----------------------------------------------------------------------
(define-public (nlp-stimuating?)
"
  Returns #t if nlp-stimuation of parse is taking place and #f if not.
"
    nlp-stimulate-parses
)

; -----------------------------------------------------------------------
(define-public (nlp-stop-stimulation)
"
  Switchs off the stimulation of WordNodes and WordInstanceNodes during parse.
"
    (set! nlp-stimulate-parses #f)
)

; -----------------------------------------------------------------------
(define (nlp-stimulate SENT STIMULUS)
"
  Stimulate the WordNodes and WordInstanceNodes associated with the SentenceNode
  SENT by STI amount.
"
    (define (stimulate x) (cog-stimulate x STIMULUS))
    (let* ((word-inst-list
                (append-map parse-get-words (sentence-get-parses SENT)))
           (word-list (append-map word-inst-get-word word-inst-list)))
        (map stimulate word-inst-list)
        (map stimulate word-list)
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

        ; Stimulate WordNodes and WordInstanceNodes
        (if nlp-stimulate-parses
            (nlp-stimulate (car sent-list) nlp-stimulation-value))

    ; Testing the Sentiment_eval function
    (cog-logger-info "nlp-parse: testing Sentiment_eval")
    (python-call-with-as "set_atomspace" (cog-atomspace))
    (cog-evaluate! (Evaluation (GroundedPredicate "py: call_sentiment_parse") (List (Node plain-text) (car sent-list))))

		; Track some counts needed by R2L.
		(r2l-count sent-list)

		; Return the sentence list.
		sent-list
	)
)

; -----------------------------------------------------------------------
