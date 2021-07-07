;
; Tools for running AIML in the AtomSpace.
;
(define-module (opencog nlp aiml))

(use-modules (srfi srfi-1) (rnrs io ports))
(use-modules (opencog) (opencog nlp) (opencog nlp oc) (opencog exec) (opencog openpsi))

; (load "aiml/bot.scm")
(load "aiml/gender.scm")
(load "aiml/subs.scm")

; ==============================================================

; Default states
(State (Concept "AIML state topic") (List))

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

(define-public (aiml-set-bot-prop PROP-FILE)
"
  set-bot-prop PROP-FILE -- Set the bot properties as defined in
  PROP-FILE

  Each of the properties corresponds to the \"bot\" keyword in AIML.

  Each line of the PROP-FILE should store a single property, in the
  form of \"key=value\".
"
	(define (set-bot STR VAL-STR)
		(State (Concept (string-append "AIML-bot-" (string-trim-both STR)))
			(string-words (string-trim-both VAL-STR))))

	(define in-port (open-file-input-port PROP-FILE))
	(define line (get-line in-port))
	(define prop (list))

	(while (not (eof-object? line))
		(set! prop (string-split line #\=))
		(set-bot (car prop) (cadr prop))
		(set! line (get-line in-port)))

	(close-port in-port)
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
; Return all of the rules that match the input SENT via a pattern
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

	(define (get-rules)
		; For getting those "wildcard" rules
		; TODO: Maybe it is better to get these rules using GetLink + SignatureLink,
		; but at the moment it does not support unordered link, and doing it this
		; way is fast...
		(define wildcard-rule-context
			(Evaluation (Predicate "*-AIML-pattern-*") (List (Glob "$star-1"))))

		(concatenate! (list
			(psi-get-dual-match SENT)
			(cog-get-root wildcard-rule-context)))
	)

	; Get all of the rules that might apply to this sentence,
	; and are inexact matches (i.e. have a variable in it)
	(filter is-usable-rule? (map gar (get-rules)))
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
	(cog-extract! maplk)
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

; Return #t if 'that' in the RULE context is actually equal
; to the current AIML that state.
(define (satisfy-that? RULE)
	(define pred (get-pred RULE "*-AIML-that-*"))
	(if (null? pred) #t
		(or (equal? (do-aiml-get (Concept "that")) (gdr pred))
			; There may be a '*' in the 'that' tag of a rule as
			; well -- use a MapLink to check the satisfiability
			(not (null? (gar (cog-execute! (MapLink (gdr pred)
				(Set (do-aiml-get (Concept "that"))))))))))
)

; AIML spec compatibility:
; AIML uses a trie, and only accepts the longest-possible match -- that is,
; it matches the 1st word, and then 2nd, 3rd... and so on until there is a
; mismatch. So for example if there are two rules
;     <pattern>THERE IS A *</pattern>
;     <pattern>THERE IS *</pattern>
; and the input is "There is a cat", it will always choose the first one
; and never choose the second. The below makes sure that OC AIML behaves
; the same way.
(define (get-longest-match RULES SENT)
	(define common 0)
	(define rtn-rules (list))

	(for-each
		(lambda (r)
			(define rule-pat (gdr (get-pred r "*-AIML-pattern-*")))
			(define cnt (list-index (lambda (x y) (not (equal? x y)))
				(cog-outgoing-set rule-pat) (cog-outgoing-set SENT)))
			; If cnt is false, that's an exact match
			(if (equal? cnt #f) (set! cnt (cog-arity SENT)))
			(cond
				((> cnt common) (set! common cnt) (set! rtn-rules (list r)))
				((= cnt common) (set! rtn-rules (append rtn-rules (list r))))))
		RULES)

	rtn-rules
)

(define-public (aiml-get-applicable-rules SENT)
"
  aiml-get-applicable-rules SENT - Get AIML rules that are suitable
  for generating a reply to the given sentence. Return pattern-based
  rules only if there are no exact matches.
"
	(define exact-rules (get-exact-rules SENT))

	(define pat-rules (get-pattern-rules SENT))

	(define top-rules (filter is-topical-rule?
		(append exact-rules pat-rules)))

	(define tht-rules
		(filter satisfy-that? top-rules))

	(get-longest-match tht-rules SENT)
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
		(define w (cog-confidence ATOM))
		(* w w)
	)

	;; Total up all of the weights on all of the rules
	;; in the RULE-LIST.
	(define sum 0.0)
	(define (add-to-sum ATOM)
		(set! sum (+ sum (get-weight ATOM))))

	;; Return #t for the first rule in the RULE-LIST for which the
	;; accumulated weight is above THRESH.
	(define accum 0.0)
	(define (pick-first ATOM THRESH)
		(set! accum (+ accum (get-weight ATOM)))
		(< THRESH accum))

	; OK, so actually total up the weights
	(for-each add-to-sum RULE-LIST)

	; Randomly pick one, using the weighting above.
	(let ((thresh (random sum)))
		(if (null? RULE-LIST)
			'()
			(let ((idx (list-index
					(lambda (ATOM) (pick-first ATOM thresh)) RULE-LIST)))
				; Returns the last rule in the list if idx is #f
				(if idx
					(list-ref RULE-LIST idx)
					(car (last-pair RULE-LIST))))
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
		(define that (do-aiml-get (Concept "that")))
		(define that-len
			(if (null? that) 0
				(string-length (string-join
					(map cog-name (cog-outgoing-set that)) " "))
			))

		; It is OK to repeat the last response if it is
		; shorter than 15 characters
		(and (equal? SENT that) (> that-len 15))
	)

	(define (do-while-same SENT CNT)
		(if (>= 0 CNT) (ListLink)
			(let ((response (get-response-step SENT)))
				(if (same-as-before? response)
					(do-while-same SENT (- CNT 1))
					response
				))))

	; Do simple substitution, for example "I'll" -> "I will"
	(set! SENT (do-aiml-subs SENT))

	; AIML pattern matching is case insensitive
	(set! SENT (List (map (lambda (w)
		(Word (string-downcase (cog-name w))))
			(cog-outgoing-set SENT))))

	(let ((response (word-list-flatten (do-while-same SENT 5))))

		; Try to look for any pickup responses if "response" is empty.
		; This may happen occasionally if it get stuck at a rule
		; that contains unsupported tags (e.g. condition), or a rule
		; that is broken in some way. Maybe it's better to trigger the
		; "pickup search" manually for now, than giving no responses
		(if (equal? response (List)) (begin
			(display "no response has been generated, looking for a pickup\n")
			(set! response (word-list-flatten
				(do-while-same (List (Glob "$star-1")) 5)))))

		; The robots response is the current "that".
		; Store up to two previous inputs and outputs
		; XXX TODO: Would be better to log and retrieve the chat
		;           history using AtTimeLink and the time server
		(if (valid-response? response)
			(let ((that (do-aiml-get (Concept "that"))))
				(if (not (null? that))
					(do-aiml-set (Concept "that-2") that))
				(do-aiml-set (Concept "that") response)))

		(let ((input (do-aiml-get (Concept "input"))))
			(if (not (null? input))
				(do-aiml-set (Concept "input-2") input))
			(do-aiml-set (Concept "input") SENT))

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
	(define reval (if (equal? '() flat-val) (WordNode "") flat-val))
	; (display "Perform AIML set key=") (display KEY) (newline)
	; (display "set value=") (display reval) (newline)
	(define rekey (Concept (string-append "AIML state " (cog-name KEY))))
	(State rekey reval)
	reval
)

; AIML-tag get -- Fetch value from a StateLink key-value pair
(DefineLink
	(DefinedSchemaNode "AIML-tag get")
	(GroundedSchemaNode "scm: do-aiml-get"))

(define-public (do-aiml-get KEY)
	(define rekey (Concept (string-append "AIML state " (cog-name KEY))))
	; gar discards the SetLink that the GetLink returns.
	(gar (cog-execute! (Get (TypedVariable (Variable "$x") (Type "ListLink"))
		(State rekey (Variable "$x"))))))

; AIML-tag bot -- Just like get, but for bot values.
(DefineLink
	(DefinedSchemaNode "AIML-tag bot")
	(GroundedSchemaNode "scm: do-aiml-bot-get"))

(define-public (do-aiml-bot-get KEY)
	(define rekey (Concept (string-append "AIML-bot-" (cog-name KEY))))
	; gar discards the SetLink that the GetLink returns.
	(gar (cog-execute! (Get (TypedVariable (Variable "$x") (Type "ListLink"))
		(State rekey (Variable "$x"))))))

(DefineLink
	(DefinedSchemaNode "AIML-tag input")
	(GroundedSchemaNode "scm: do-aiml-input"))

(define-public (do-aiml-input idx)
	(if (= (cog-number idx) 1)
		(do-aiml-get (Concept "input"))
		(do-aiml-get (Concept (string-append "input-"
			(car (string-split (cog-name idx) #\.))))))
)

(DefineLink
	(DefinedSchemaNode "AIML-tag that")
	(GroundedSchemaNode "scm: do-aiml-that"))

(define-public (do-aiml-that idx)
	(if (= (cog-number idx) 1)
		(do-aiml-get (Concept "that"))
		(do-aiml-get (Concept (string-append "that-"
			(car (string-split (cog-name idx) #\.))))))
)

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
