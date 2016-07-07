;
; Tools for running AIML in the AtomSpace.
;
(define-module (opencog nlp aiml))

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog nlp) (opencog exec) (opencog openpsi))

; (load "aiml/bot.scm")
(load "aiml/gender.scm")

; ==============================================================

(define-public (token-seq-of-parse PARSE)
"
  token-seq-of-parse PARSE -- Create a list of words from input parse.

  PARSE is assumed to be a ParseNode, pointing to text that has been
  processed by RelEx.

  Example:
     (relex-parse \"I love you\")
     (map token-seq-of-parse
         (sentence-get-parses (car (get-new-parsed-sentences))))
"

	(Evaluation
		(PredicateNode "Token Sequence")
		PARSE
		(ListLink
			(remove null?
				(map word-inst-get-lemma (parse-get-words-in-order PARSE)))
		))
)

; ==============================================================

(define-public (token-seq-of-sent SENT)
"
  token-seq-of-sent -- Create a list of words from input sentence.

  SENT is assumed to be a SentenceNode, pointing to text that has been
  processed by RelEx.

  Example:
     (relex-parse \"I love you\")
     (token-seq-of-sent (car (get-new-parsed-sentences)))

  will create the following output:

     (Evaluation
        (PredicateNode \"Token Sequence\")
        (Parse \"sentence@3e975d3a-588c-400e-a884-e36d5181bb73_parse_0\")
        (List
           (Concept \"I\")
           (Concept \"love\")
           (Concept \"you\")
        ))
"
	(map token-seq-of-parse (sentence-get-parses SENT))
)

; ==============================================================

(define (string-words SENT-STR)
"
  string-words SENT-STR -- chop up SENT-STR string into word-nodes
  CAUTIION: this by-passes the NLP pipeline, and instead performs
  an extremely low-brow tokenization!  In particular, it does not
  handle upper/lower case correctly (all AIML rules are lower-case
  only) and it does not handle punctuation (AIML rules have no
  punctuation in them).

  This is a debugging utility, and not for general use!  It is NOT
  exported for public use!

  Example:
     (string-words \"this is a test\")
  produces the output:
     (List (Word \"this\") (Word \"is\") (Word \"a\") (Word \"test\"))

"
	(ListLink (map
		(lambda (w) (Word w))
		(string-tokenize SENT-STR)))
)

; --------------------------------------------------------------

(define (word-list-flatten LIST)
"
  word-list-flatten LIST -- remove nested ListLinks

  This is used for flattening out the assorted lists of words
  that result from the AIML processing utilities.

  This is not exported publicly, so as to avoid polluting the
  namespace.

  Example: the input
     (List (Concept \"A\") (List (Concept \"B\") (Concept \"C\")))
  will be flattened to:
     (List (Concept \"A\") (Concept \"B\") (Concept \"C\"))
"
	(define (unwrap ATOM)
		(if (equal? 'ListLink (cog-type ATOM))
			(concatenate (map unwrap (cog-outgoing-set ATOM)))
			(list ATOM)))

	(if (equal? 'ListLink (cog-type LIST)) (ListLink (unwrap LIST)) LIST)
)

; --------------------------------------------------------------

(define (word-list-set-flatten SET)
"
  word-list-set-flatten LIST -- flatten all word-lists in a set.

  This is not exported publically, so as to avoid pullotuing the
  namespace.

  Example: the input
     (Set (List (Concept \"A\") (List (Concept \"B\") (Concept \"C\"))))
  will be flattened to:
     (Set (List (Concept \"A\") (Concept \"B\") (Concept \"C\")))
"
	(cond
		((equal? 'ListLink (cog-type SET))
			(word-list-flatten SET))
		((equal? 'SetLink (cog-type SET))
			(SetLink (map word-list-flatten (cog-outgoing-set SET))))
	)
)

; --------------------------------------------------------------
; non-public utilities

; Return #t if the rule is a AIML chat rule
(define (chat-rule? r)
	(equal? (gdr r) (Concept "AIML chat subsystem goal")))

; Given an AIML rule, return a scheme list holding the context
; followed by the action.  Given a rule, (gar r) is the AndLink.
; Either gaar or gadr is the context; the other is the action.
; We identify the action by using the `psi-action?` utility.
; XXX FIXME. This is yucky, something prettier is needed.
(define (get-ctxt-act r)
	(define andy (gar r))
	(define pa (gar andy))
	(define pb (gdr andy))
	(if (psi-action? pb)
		(list pa pb)
		(list pb pa))
)

; Get the predicate named PRED out of the context part of the rule.
; Assumes that there is only one such.
(define (get-pred RULE PRED-NAME)
	(define ctxt (car (get-ctxt-act RULE)))
	(define (is-pred EVL)
		(equal? PRED-NAME (cog-name (gar EVL))))
	(define pred-list (filter is-pred (cog-outgoing-set ctxt)))
	(if (null? pred-list) '() (car pred-list))
)

; -----------------------------------------------------------
; Return all of the rules that provide exact matches to the input
; sentence.
(define (get-exact-rules SENT)
	; Verify that RULE really is an exact rule for this sentence.
	; -- check that its a chat rule
	; -- check that it has the AIML pattern PredicateNode
	; -- check that the pattern is the sentence.
	(define (is-exact-rule? RULE)
		(if (not (chat-rule? RULE)) #f
			(let ((pred (get-pred RULE "*-AIML-pattern-*")))
				(if (null? pred) #f
					(equal? (gdr pred) SENT)))))

	;; XXX TODO -- filter out the exact rules that have non-trivial
	;; THAT and TOPIC contexts.

	; Get all the exact rules that apply to the SENT
	(filter is-exact-rule?
		(map gar (psi-get-exact-match SENT)))
)

; Run the exact rules.  We already know that the context is
; fulfilled, so just grab the action, and run it.
(define (run-exact-rule RULE)
	(cog-execute! (cadr (get-ctxt-act RULE)))
)

; -----------------------------------------------------------
; Return all of the rules that match the input SENT via a patern
; (i.e. use variables in the pattern)
(define (get-pattern-rules SENT)

	; Make sure that a given pattern RULE (i.e. a rule with
	; variables in it) can actually result in a match on the
	; sentence.
	(define (is-usable-rule? RULE)
		(if (not (chat-rule? RULE)) #f
			(let ((pred (get-pred RULE "*-AIML-pattern-*")))
				(if (null? pred) #f
					(not (null? (gar
						(cog-execute! (MapLink (gdr pred) (SetLink SENT)))
				))))
		)))

	; Get all of the rules that might apply to this sentence,
	; and are inexact matches (i.e. have a variable in it)
	(filter is-usable-rule?
		(map gar (psi-get-dual-match SENT)))
)

; Given a pattern-based rule, run it. Given that it has variables
; in it, accomplish this by creating and running a MapLink.
; XXX Need to handle that, topic rules as appropriate.
(define (run-pattern-rule RULE SENT)
	(define maplk (MapLink
		(ImplicationLink
			(gdr (get-pred RULE "*-AIML-pattern-*"))
			(cadr (get-ctxt-act RULE)))
		(SetLink SENT)
	))
	(define results (cog-execute! maplk))

	; Remove the bindlink, to avoid garbaging up the atomspace.
	(cog-delete maplk)
	results
)

; --------------------------------------------------------------

; Return #t if the topic in the RULE context is actually equal
; to the current AIML topic state.
; XXX FIXME -- handle topic stars also ....
(define (is-topical-rule? RULE)
	(define pred (get-pred RULE "*-AIML-topic-*"))
	(if (null? pred) #t
		(equal?
			(do-aiml-get (Concept "topic"))
			(gdr pred)
		)
	)
)

;; XXX FIXME Handle THAT sections too ... same way as topic ...
(define-public (aiml-get-applicable-rules SENT)
"
  aiml-get-applicable-rules SENT - Get all AIML rules that are suitable
  for generating a reply to the givven sentence.
"
	(define all-rules
		(concatenate! (list
			(get-exact-rules SENT)
			(get-pattern-rules SENT))))

	(define top-rules
		(filter is-topical-rule? all-rules))

	top-rules
)

; --------------------------------------------------------------

(define-public (aiml-select-rule RULE-LIST)
"
  aiml-select-rule RULE-LIST - Given a list of AIML rules,
  select one to run.
"
	;; XXX FIXME crazy hacky weight-adjusting formula. This makes
	;; no sense at all, but is a hacky hack designed to pick more
	;; desirable rules more often.  Someone should figure out
	;; some weighting formula that makes more sense tahn this.
	;; Currently, the square of the confidence.
	(define (get-weight ATOM)
		(define w (tv-conf (cog-tv ATOM)))
		(* w w)
	)

	;; Total up all of the weights on all of the rules
	;; in the RULE-LIST.
	(define sum 0.0)
	(define (add-to-sum ATOM)
		(set! sum (+ sum (get-weight ATOM))))

	;; Return #t for the first rule in the RULE-LIST for which the
	;; accumulated weight is above THRESH.
	(define accum 0.000000001)
	(define (pick-first ATOM THRESH)
		(set! accum (+ accum (get-weight ATOM)))
		(< THRESH accum))

	; OK, so actually total up the weights
	(for-each add-to-sum RULE-LIST)

	; Randomly pick one, using the weighting above.
	(let ((thresh (random sum)))
		(if (null? RULE-LIST)
			'()
			(list-ref RULE-LIST (list-index
				(lambda (ATOM) (pick-first ATOM thresh)) RULE-LIST))
	))
)

; --------------------------------------------------------------

(define-public (aiml-run-rule SENT RULE)
"
  aiml-run-rule - Given a single AIML RULE, and the SENT sentence to
  apply it to, run the rule, and return the response.
"
	(define (is-exact-rule? RULE)
		(let ((pred (get-pred RULE "*-AIML-pattern-*")))
			(if (null? pred) #f
				(equal? (gdr pred) SENT))))

	(if (null? RULE)
		(ListLink)
		(if (is-exact-rule? RULE)
			(run-exact-rule RULE)
			(run-pattern-rule RULE SENT)
	))
)

; --------------------------------------------------------------

; Return true if RESP is a SetLink containing a non-empty
; word sequence. ... or if RESP is just a ListLink that isn't
; empty (we assume the ListLink is just a single sentence).
; Examples of valid responses are:
;    (ListLink (Word "blah") (Word "blah"))
;    (SetLink (ListLink (Word "blah") (Word "blah")))
; Examples of invalid responses:
;    (ListLink)
(define (valid-response? RESP)
	(if (equal? 'SetLink (cog-type RESP))
		(if (null? (gar RESP)) #f
			(not (null? (gaar RESP))))
		(not (null? (gar RESP)))))

(define selected-rule "")
(define-public (aiml-get-selected-rule)
"
  aiml-get-selected-rule - Return the AIML rule that has been selected
  by the engine
"
	selected-rule
)

;; get-response-step SENT -- get an AIML response to the sentence
;; SENT.  Recursive, i.e. it will recursively handle the SRAI's,
;; but is not necessarily the outermost response generator. That
;; is, this is the correct routine to call for handling SRAI
;; recursion.
(define (get-response-step SENT)
	(define all-rules (aiml-get-applicable-rules SENT))

	; Some AIML rules fail to generate any response at all --
	; These are typically srai rules that fail to terminate.
	; So, try again, picking a different rule, till we do get some
	; response.
	(define (do-while-null SENT CNT)
		(if (>= 0 CNT) (ListLink)
			(let* ((rule (aiml-select-rule all-rules))
					(response (aiml-run-rule SENT rule)))
				(if (valid-response? response)
					(begin (set! selected-rule rule) response)
					(do-while-null SENT (- CNT 1))
				))))

	(let ((response (do-while-null SENT 10)))
		; Strip out the SetLink, if any.
		(if (equal? 'SetLink (cog-type response))
			(set! response (gar response)))

		; Return the response.
		(word-list-flatten response)
	)
)

(define-public (aiml-get-response-wl SENT)
"
  aiml-get-response-wl SENT - Get AIML response to word-list SENT
"
	; Sometimes, the mechanism will result in exactly the same
	; response being given twice in a row. Avoid repeating, by
	; checking to see if the suggested response is the same as the
	; previous response. Right now, we just check one level deep.
	; XXX FIXME .. Maybe check a much longer list??
	(define (same-as-before? SENT)
		(equal? SENT (do-aiml-get (Concept "that")))
	)

	(define (do-while-same SENT CNT)
		(if (>= 0 CNT) (ListLink)
			(let ((response (get-response-step SENT)))
				(if (same-as-before? response)
					(do-while-same SENT (- CNT 1))
					response
				))))

	(let ((response (word-list-flatten (do-while-same SENT 5))))

		; The robots response is the current "that".
		(if (valid-response? response)
			(do-aiml-set (Concept "that") response))

		; Return the response.
		response
	)
)

; --------------------------------------------------------------

; AIML-tag srai -- Run AIML recursively
; srai == "stimulous-response ai"
(DefineLink
	(DefinedSchemaNode "AIML-tag srai")
	(GroundedSchemaNode "scm: do-aiml-srai"))

(define-public (do-aiml-srai x)
	(display "perform srai recursion\n") (display x) (newline)
	(let ((resp (get-response-step (word-list-flatten x))))
		(display "srai result is\n") (display resp) (newline)
		resp
	)
)

; AIML-tag think -- execute the arguments, silently.
(DefineLink
	(DefinedSchemaNode "AIML-tag think")
	(GroundedSchemaNode "scm: do-aiml-think"))

; Because do-aiml-think is a black-box, its arguments are
; force-evaluated by teh evaluator. Thus, there is nothing to
; do with the argument, when we get here.  Don't return anything
; (be silent).
(define-public (do-aiml-think x)
	; (display "Perform think\n") (display x) (newline)
	; 'think' never returns anything
	'()
)

; AIML-tag set -- Use a StateLink to store a key-value pair
(DefineLink
	(DefinedSchemaNode "AIML-tag set")
	(GroundedSchemaNode "scm: do-aiml-set"))

(define-public (do-aiml-set KEY VALUE)
	(define flat-val (word-list-flatten VALUE))
	; (display "Perform AIML set key=") (display KEY) (newline)
	; (display "set value=") (display flat-val) (newline)
	(define rekey (Concept (string-append "AIML state " (cog-name KEY))))
	(State rekey flat-val)
	flat-val
)

; AIML-tag get -- Fetch value from a StateLink key-value pair
(DefineLink
	(DefinedSchemaNode "AIML-tag get")
	(GroundedSchemaNode "scm: do-aiml-get"))

(define-public (do-aiml-get KEY)
	(define rekey (Concept (string-append "AIML state " (cog-name KEY))))
	; gar discards the SetLink that the GetLink returns.
	(gar (cog-execute! (Get (State rekey (Variable "$x"))))))

; AIML-tag bot -- Just like get, but for bot values.
(DefineLink
	(DefinedSchemaNode "AIML-tag bot")
	(GroundedSchemaNode "scm: do-aiml-bot-get"))

(define-public (do-aiml-bot-get KEY)
	(define rekey (Concept (string-append "AIML-bot-" (cog-name KEY))))
	; gar discards the SetLink that the GetLink returns.
	(gar (cog-execute! (Get (State rekey (Variable "$x"))))))

;; -------------------------
; AIML-tag person -- Convert 1st to third person, and back.
(DefineLink
	(DefinedSchemaNode "AIML-tag person")
	(GroundedSchemaNode "scm: do-aiml-person"))

; AIML-tag person2 -- Convert 1st to second person, and back.
(DefineLink
	(DefinedSchemaNode "AIML-tag person2")
	(GroundedSchemaNode "scm: do-aiml-person2"))

; AIML-tag gender -- Convert male to female and back.
(DefineLink
	(DefinedSchemaNode "AIML-tag gender")
	(GroundedSchemaNode "scm: do-aiml-gender"))

; AIML-tag formal -- Do nothing, its pointless for spoken text.
(DefineLink
	(DefinedSchemaNode "AIML-tag formal")
	(GroundedSchemaNode "scm: do-aiml-formal"))

(define-public (do-aiml-formal x) x)

; ==============================================================

(load "aiml/bot.scm")

;; mute the thing
*unspecified*
