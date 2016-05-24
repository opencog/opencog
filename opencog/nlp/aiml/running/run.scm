;
; Tools for running AIML in the AtomSpace.
;

(use-modules (srfi srfi-1))
(use-modules (opencog) (opencog nlp))

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

; --------------------------------------------------------------

(define-public (aiml-get-response-wl SENT)
"
  aiml-get-response-wl SENT - Get AIML response to word-list SENT
"

	; Return #t if the rule is a aiml chat rule
	(define (chat-rule? r)
		(equal? (gdr r) (Concept "AIML chat goal")))

	; Create a BindLink
	(define (run-rule r)
(display "duuude run rule \n") (display r) (newline)
		(cog-execute! (Map (Implication (gaar r) (gdar r)) SENT)))

	; for now, just get the responses.
	(map run-rule
		(filter chat-rule? (psi-get-dual-rules SENT)))

)

; --------------------------------------------------------------

(DefineLink
	(DefinedSchemaNode "AIML-tag srai")
	(GroundedSchemaNode "scm: do-aiml-srai"))

; Run AIML recursively
(define-public (do-aiml-srai x)
	(display "duuude srai recurse\n") (display x) (newline)
	(aiml-get-response-wl x)
)

(DefineLink
	(DefinedSchemaNode "AIML-tag think")
	(GroundedSchemaNode "scm: do-aiml-think"))

(define-public (do-aiml-think x)
	(display "duuude think\n") (display x) (newline)
	x
)

(DefineLink
	(DefinedSchemaNode "AIML-tag set")
	(GroundedSchemaNode "scm: do-aiml-set"))

(define-public (do-aiml-set KEY VALUE)
	(display "duuude set\n") (display KEY) (newline)
	(display VALUE) (newline)
	VALUE
)

; ==============================================================
