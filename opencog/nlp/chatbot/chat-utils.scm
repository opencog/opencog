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
(define-public (get-previous-said-sents from-time)
"
  Returns a list of SentenceNodes that are inputed after the given time.

  from-time:
  - The time in seconds since 1970-01-01 00:00:00 UTC. (current-time) gives
    such time.
"
; TODO use the timeserver when it is ready.
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

    (define result '())
    (define (last-sent sent)
        (let ((sent-time (string->number (cog-name (gar sent)))))
            (if (>= sent-time from-time)
                (set! result (append result (list (gdr sent))))
            )
        ))

    (let ((sents (cog-execute! query)))
        (for-each last-sent (cog-outgoing-set sents))
        (cog-delete sents)
        result
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
		(error "Please enter a valid sentence, not an empty string"))

	; Call the RelEx server
	(catch #t (lambda () (relex-parse plain-text))
		(lambda (key . rest)
			(display "Error: Cannot connect to RelEx server: ")
			(display key) (newline) (display rest) (newline)))

	(if (null? (get-new-parsed-sentences))
		(error "The RelEx server seems to have crashed!"))

	(let* ((sent-list (get-new-parsed-sentences))
			(sent-node (car sent-list)))

		; Unhook the anchor. MUST do this before r2l-parse, as
		; otherwise, parse-get-relex-outputs will wrap it in a
		; SetLink! Ouch!!
		(release-new-parsed-sents)

		; Tage the sentence with the wall-clock time.
		(sent-set-time sent-node)

		; Perform the R2L processing.
		(r2l-parse sent-node)

		; Stimulate WordNodes and WordInstanceNodes
		(if nlp-stimulate-parses
			(nlp-stimulate sent-node nlp-stimulation-value))

		; Call the Sentiment_eval function
		(cog-logger-info "nlp-parse: testing Sentiment_eval")
		(python-call-with-as "set_atomspace" (cog-atomspace))
		(cog-evaluate!
			(Evaluation
				(GroundedPredicate "py: call_sentiment_parse")
				(List (Node plain-text) sent-node)))

		; Track some counts needed by R2L.
		(r2l-count sent-list)

		; Return the sentence list.
		sent-list
	)
)

; -----------------------------------------------------------------------
