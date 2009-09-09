;
; chat-interface.scm
;
; Simple scheme interface to glue together the chat-bot to the cog-server.
; 
; Linas Vepstas April 2009
;

(use-modules (ice-9 rdelim))  ; for the system call

;; Hack to flush IO except this hack doesn't work :-(
;; doesn't work because of how the SchemEval.cc handles ports ... 
(define (fflush) (force-output (car  (fdes->ports 1))))

; -----------------------------------------------------------------------
; Debug printing support.  Turnging on debug printing will prevent
; output from being sent to the chat room.  It is still printed by the
; chatbot to its stdout, and so can be debugged there. The chat client
; explicitly looks for ":dbg" and ":end-dbg" in the chat stream.
;
(define (dbg-display x)
	(display "\n:dbg\n")
	(display x)
)
(define (end-dbg-display)
	(display "\n:end-dbg\n")
)

; -----------------------------------------------------------------------
; chat-return -- throw a return to the chat client.
;
; This is a pseudo-multi-threading hack. The core problem here is
; the command-response nature of the current opencog server design,
; coupled to the fact that no response is generated until the 
; command completes, and that no response is possible after the 
; command completes. So instead, the chat server needs to poll
; the opencog server to see if there's more output expected.
;
; It would be nice to use call-with-current-continuation but 
; unfortuantely, scm_with_guile() erects a continuation barrier,
; making this hard/impossible. Argh.
;
; So instead, what's implemented here is throw-catch semantics.
; An exceptiuon is thrown, its caught by the guile interpreter, 
; and thus passed on to the chat client.  Because this is a throw,
; it can be used anywhere in the code to break out of evaluation.
;
; The chat client is looking for "\n:scm hush\r (scheme code)\n".
; If it sees this, then the (scheme code) is resubmitted to opencog.
; The "\n:scm" and the "\r" are both important parts of the syntax,
; don't mess them up.

(define (chat-return x)
	(throw 'cog-yield 
		(string-join 
			; Must use \r here, not \n, as else the chatbot will screw up.
			(list "\n:scm hush\r "
				(call-with-output-string (lambda (y) (display x y)))
				"\n"
			)
		)
	)
)

; -----------------------------------------------------------------------
; Attach parses to a place where they can be found.
;
(define *new-parses-anchor* (AnchorNode "# NEW PARSES" (stv 1 1)))
(define (attach-new-parses sent-list)
   (attach-parses-to-anchor sent-list *new-parses-anchor*)
)
(define (release-new-parses)
	(release-from-anchor *new-parses-anchor*)
)

; -----------------------------------------------------------------------
; Anchor for truth-assertion relations. These need to be promoted to semes.

(define *truth-assertion-anchor* (AnchorNode "# TRUTH ASSERTION" (stv 1 1)))

(define (get-truth-assertions)
	(define (get-verbs)
		(cog-chase-link 'ListLink 'WordInstanceNode *truth-assertion-anchor*)
	)
	(concatenate! (map word-inst-get-head-relations (get-verbs)))
)

(define (release-truth-assertions)
	(release-from-anchor *truth-assertion-anchor*)
)

; -----------------------------------------------------------------------
; Anchor for truth-query parses. The things that get attached to this
; anchor are ParseNodes of parses that seem to be (as yet unasnwered) 
; truth-query questions.
;
(define *truth-query-anchor* (AnchorNode "# TRUTH QUERY" (stv 1 1)))

(define (get-truth-queries)
	(cog-chase-link 'ListLink 'ParseNode *truth-query-anchor*)
)

(define (release-truth-query-anchor)
	(release-from-anchor *truth-query-anchor*)
)

; -----------------------------------------------------------------------
; chat-get-simple-answer -- get single-word replies to a question.
;
; This is a super-dooper cheesy way of reporting the answer to a question.
; Right now, its looks for Nodes attached, via ListLink, to an 
; AnchorNode called "# QUERY SOLUTION". Some other stage of processing 
; needs to have attached the nodes to this anchor.
;
; Anyway, three different kinds of things can be found:
; WordNodes, almost always "yes", in answer to a yes/no question.
; WordInstanceNodes, in answer to simple pattern matching.
; SemeNodes, in answer to triples matching.
; A list of these "answers" is returned.
; 
(define *query-soln-anchor* (AnchorNode "# QUERY SOLUTION"))
(define (chat-get-simple-answer)
	(define (do-one-answer answ)
		(let ((tipo (cog-type answ)))
			(cond 
				((eq? tipo 'WordNode) answ)
				((eq? tipo 'WordInstanceNode) (word-inst-get-lemma answ))
				((eq? tipo 'SemeNode) 
					; the lemma-link for a seme might still be sitting on disk!
					(load-referers answ)
					(word-inst-get-lemma answ)
				)

				; Explicitly ignore VariableNodes, these can be there
				; due to ImplicationLinks ... 
				((eq? tipo 'VariableNode) '())
				(else '())
			)
		)
	)
	(delete-duplicates! ;; remove dup answers
		(remove! null?     ; remove nulls
			(map            ; get each answer ... 
				(lambda (x) (do-one-answer (cadr (cog-outgoing-set x)))) 
				(cog-incoming-set *query-soln-anchor*)
			)
		)
	)
)
(define (chat-release-simple-answer)
	(release-from-anchor *query-soln-anchor*)
)

; -----------------------------------------------------------------------
; chat-prt-soln -- print a list of words as the answer to a question
;
; Typically, this routine will be used to print single-word replies
; e.g. "yes", but sometimes a list of words shows up for extensive
; questions (e.g. "what is an instrument" lists many instruments)
;
(define (chat-prt-soln msg soln-list)

	(define (prt-one item)
		(if (null? item)
			(display "(null)")
			(display (cog-name item))
		)
		(display " ")
	)

	(display msg)
	(cond
		((null? soln-list) '())
		((list? soln-list) (for-each prt-one soln-list))
		((string? soln-list) (display sln-list))
	)
)

; -----------------------------------------------------------------------
; say-id-english -- process user input from chatbot.
; args are: user nick from the IRC channel, and the text that the user 
; entered.
;
; IMPORTANT NOTE: To get semi-interactive I/O while potentially lengthly
; processing is going on, the processing as been split into stages,
; designed so that each stage returns from the scheme interpreter,
; with some output for the chatbot.  The next stage of processing can
; then be continued by 
; Or use (chat-return ) instead.
;
; This interactive design is not very pretty, and it would be better
; to come up with some sort of multi-threaded design, with one of the 
; threads listening on the scheme port.  FIXME XXX This should be fixed,
; because the current approach has *many* problems. However, the fix is
; hard, so we just punt for now.
;
(define (say-id-english nick txt)

	; Declare some state variables for the imperative style to follow
	(define sents '())
	(define is-question #f)

	; Parse the input, load it into opencog.
	(relex-parse txt)
	(set! sents (get-new-parsed-sentences))

	; Hmm. Seems like sents is never null, unless there's a 
	; programmig error in RelEx.  Otherwise, it always returns 
	; something, even if the input was non-sense.
	(if (null? sents)
		(let ()
			(display "Hello ")
			(display nick)
			(display ", you said: \"")
			(display txt)
			(display "\" but I couldn't parse that.")
			(newline)
			(chat-return "(say-final-cleanup)")
		)
	)

	; Perform a simple pattern matching to the syntactic
	; form of the sentence.
	(set! is-question (cog-ad-hoc "question" (car sents)))

	;; Was a question asked?
	(if is-question 
		(let ((ans (chat-get-simple-answer)))
			(display "Hello ")
			(display nick)
			(display ", you asked a question: ")
			(display txt)
			(newline)

			; If pattern-matching found an answer, print it.
			(if (not (null? ans))
				(let () 
					(chat-prt-soln "Syntax pattern match found: " ans)
					(chat-return "(say-final-cleanup)")
				)
			)
		)
		(let ()
			(display "Hello ")
			(display nick)
			(display ", you made a statement: ")
			(display txt)
			(newline)
			(chat-return "(say-declaration)")
		)
	)

	; If we are here, a question was asked, and syntax matching
	; did not provide an answer.
	(chat-return "(say-try-truth-query)")
)

; -----------------------------------------------------------------------
; say-declaration -- User made a declaration. Perform processing
; of declarative statements.
;
; We assume that all declarations are true; therefore, immediately
; promote them to semes.

(define (say-declaration)
(dbg-display "entering declaration processing\n")

	; Run the triples processing.  
	(attach-sents-for-triple-processing (get-new-parsed-sentences))
	(create-triples)
	(dettach-sents-from-triple-anchor)

	; Promote triples to semes.
	(let* ((trips (get-new-triples))
		(trip-semes (promote-to-seme same-modifiers-promoter trips))
		)
(dbg-display "statement triple semes are:\n")
(display trip-semes)
(end-dbg-display)
	)

	; Look for truth assertions, and promote those to semes.
	; By "truth assertion" we mean "subject verb object" sentences
	; that do not have any prepositions in them.
	;
	; We don't want to just promote any-old relex relations to semes, 
	; as that will create all sorts of havoc. We only want to promote
	; actual subject-verb-object assertions, so that we can answer
	; truth-query questions on them. 
	(attach-new-parses (get-new-parsed-sentences))
	(find-truth-assertions)

(dbg-display "truth-assertions are:\n")
(display 
	(promote-to-seme same-modifiers-promoter (get-truth-assertions))
)
(end-dbg-display)

	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; anchor-bottom-side -- anchor triples for pattern matching.
;
; In order for the pattern matcher to locate things attached to
; anchors, they need to be attached in a way to "keep things simple"
; The simplest way to do this seems to be to attach one of the
; parts of the triple to the anchor.  e.g. given the triple
; "made_from(clay,pottery)" this attaches "clay" to the anchor.
;
; The question-answering rules look for things on this anchor.
;
(define *bottom-anchor* (AnchorNode "# TRIPLE BOTTOM ANCHOR"))
(define (anchor-bottom-side trip-list)
	(define (re-anchor-one trip)
		(let* ((ll (cadr (cog-outgoing-set trip)))
				(os (cog-outgoing-set ll))
				)
			(ListLink (stv 1 1) *bottom-anchor* (car os))
			(ListLink (stv 1 1) *bottom-anchor* (cadr os))
		)
	)
	(map re-anchor-one trip-list)
(dbg-display "bottumsup\n")
(display (cog-incoming-set *bottom-anchor*))
(newline)
(end-dbg-display)
)

(define (release-bottom-anchor)
	(release-from-anchor *bottom-anchor*)
)

; -----------------------------------------------------------------------
; Loop over a list of question rules, one by one. Stop iterating upon
; the first one that returns an answer.
;
(define (loop-over-questions q-list)
	(define (do-one-question quest)

		; The do-varscope returns a ListLink. We need to nuke
		; that, as otherwise it will only cause trouble later.
		(let ((rslt (cog-ad-hoc "do-varscope" quest)))
(dbg-display "q-apply result is\n")
(display rslt)
(end-dbg-display)
			(cog-delete rslt)
		)

		; return #t if the rule found an answer
		(not (null? (chat-get-simple-answer)))
	)
	; "any" will call "do one question" on each rule, until
	; one of them returns #t (i.e. when an answer is found)
	(any do-one-question q-list)
)

; -----------------------------------------------------------------------
; say-try-truth-query -- try answering a yes/no question
; Some questions are truth queries, posing a hypothesis, and asking for a 
; yes/no answer. This handles these.
;
; XXX FIXME: this should be called only if we catually *have* a 
; truth query. And if we do have a truth query, then we should NOT
; do the triples pipeline.
;
(define (say-try-truth-query)

	(attach-new-parses (get-new-parsed-sentences))

	; Apply a set of rules to determin if we even *have* a truth query.
	(for-each
		(lambda (rule) 
			(cog-delete ; need to delete the returned ListLink
				(cog-ad-hoc "do-varscope" rule)
			)
		) 
		*truth-query-id-list*
	)

	; If this does not apear to be a truth query, then cut out
	; of here, and try something else.
	(if (null? (get-truth-queries))
		(let ()
			(dbg-display "No truth-query found, try triples-qa.\n")
			(end-dbg-display)
			(chat-return "(say-try-triple-qa)")
		)
	)

	; Hmm promote trip semes ?? -- later 
	; Also -- cannot blithly promote, do *only* the semes
	; but not the triples themselves
	;	(trip-semes (promote-to-seme same-modifiers-promoter trips))
	;

	; First, we want to grab all of the words in the question, and
	; tag them with semes. (XXX We don't really need all the words,
	; only the words that are participating in relations.)
	(for-each same-modifiers-promoter
		(concatenate! 
			(map parse-get-words
				(sent-list-get-parses (get-new-parsed-sentences))
			)
		)
	)
	
	; Try each truth-query template ... 
	(loop-over-questions *truth-query-rule-list*)

	(let ((ans (chat-get-simple-answer)))

		(if (null? ans)
			; No answer, tell them so.
			(chat-prt-soln "Truth query determined \"no\". " '())

			; The answer must be yes.
			(chat-prt-soln "Truth query determined \"yes\": " ans)
		)
	)

	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; say-try-triple-qa -- try answering questions using triples 
; The first part, "say-id-english", parsed the user input, and 
; attempted a syntax-pattern match to any questions.  If that fails,
; this is called.
;
(define (say-try-triple-qa)

	; Run the triples processing.  
	(attach-sents-for-triple-processing (get-new-parsed-sentences))
	(create-triples)
	(dettach-sents-from-triple-anchor)

   ; Try again with pattern matching on the triples.
	(let ((trips (get-new-triples)))
		(if (null? trips)
			(let ()
				(dbg-display "No question triples found, try deduction.\n")
				(end-dbg-display)
				(chat-return "(say-try-deduction)")
			)
		)

		; The question-rule-x looks for stuff attached to this anchor
		(anchor-bottom-side trips)

		; Pull in any semes that might be related...
		(fetch-related-semes trips)

(dbg-display "duude question trips are:\n")
(display trips)
(end-dbg-display)

		; Now try to find answers to the question
		(loop-over-questions *question-rule-list*)

		(let ((ans (chat-get-simple-answer)))

			(if (null? ans)
				(let ()
					(dbg-display "No asnwer triples found, try deduction.\n")
					(end-dbg-display)
					(chat-return "(say-try-deduction)")
				)
				(let ()
					; print, and skip to the end of processing
					(chat-prt-soln "Triples abstraction found: " ans)
					(chat-return "(say-final-cleanup)")
				)
			)
		)
	)
	; Can't reach here, above should have handled all returns
)

; -----------------------------------------------------------------------
; say-try-deduction -- try answering question via basic deduction
; This attempts to do some basic deduction
;
(define (say-try-deduction)

	; If we still don't have an answer, try making a deduction
	; (this is an extremely simple-minded hack right now)
	; First, the question needs to be promoted to semes,
	(let* ((trips (get-new-triples))
			(trip-semes (promote-to-seme same-modifiers-promoter trips))
		)
		(if (not (null? trip-semes))
			(let* ((ftrip (car trip-semes))
					(ans (cog-ad-hoc "do-varscope" (make-simple-chain ftrip)))
				)
(dbg-display "duude question-deduct trip-semes are:\n")
(display trip-semes)
(newline)
(display "and the chain is\n")
(display (make-simple-chain ftrip))
(display "and the ans is is\n")
(display (cog-outgoing-set ans))
(end-dbg-display)
				(if (and ans (not (null? (cog-outgoing-set ans))))
					(display "Yes.\n")
					(display "Simple deduction found no answer.\n")
				)

				(if ans
					(cog-delete ans)
				)
			)
		)
	)

	(display "Deduction step found no answer.\n")
	; Call the final stage
	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; say-final-cleanup -- run cleanup of the chat processing
;
(define (say-final-cleanup)

	(dbg-display "Cleaning up now\n")

	; un-anchor assorted items
	(release-bottom-anchor)
	(release-new-parses)

	; Delete list of triples, so they don't interfere with the next question.
	(release-result-triples)
	(release-truth-assertions)
	(release-truth-query-anchor)

	; Delete  the list of solutions, so that we don't accidentally
	; replay it when the next question is asked.
	(chat-release-simple-answer)

	; cleanup -- these sentences are not new any more
	(release-new-parsed-sents)
	""
)

; -----------------------------------------------------------------------
