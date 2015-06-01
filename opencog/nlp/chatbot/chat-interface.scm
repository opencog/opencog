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
; An exception is thrown, its caught by the guile interpreter, 
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

(define (get-new-parses)
	(cog-chase-link 'ListLink 'ParseNode *new-parses-anchor*)
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
; get-truth-queries -- return a list of new parses that are truth queries
; These must have been previously tagged as such by running appropriate
; pattern matches.
; 
(define (get-parses-with-property prop-name)
	(define prop (DefinedLinguisticConceptNode prop-name))
	(remove! null?
		(map
			(lambda (parse) (cog-link 'InheritanceLink parse prop))
			(get-new-parses)
		)
	)
)

(define (get-truth-queries)
	(get-parses-with-property "truth-query")
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
;     XXX except that above isn't done any more -- WordNodes are
;     never attached, it seems. XXX
; WordInstanceNodes, in answer to SVO pattern matching.
; SemeNodes, in answer to triples matching.
;
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
					; The lemma-link for a seme might still be sitting on disk!
					; We will need the lemma when printing, etc.
					; XXX should we defer this loading till later ?? 
					(load-referers answ)
					answ
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

	; Get the lemma of the word or seme instance, print it.
	(define (prt-lemma winst)
		(display (cog-name (word-inst-get-lemma winst)))
		(display " ")
	)

	; Print lemma of word or seme instance.
	; If word or seme inst has modifiers, then print those first
	(define (prt-with-mods winst)
		(let ((mods (noun-inst-get-relex-modifiers winst))
			)
			(if (not (null? mods))
				(for-each prt-lemma
					(map relation-get-dependent mods)
				)
			)
			(prt-lemma winst)
		)
	)

	; winst might be a word-instance or a seme
	(define (prt-one winst)
		(if (null? winst)
			(display "(null) ")
			(prt-with-mods winst)
		)
	)

	; First display the message, then display the rest.
	(display msg)
	(cond
		((null? soln-list) '())
		((list? soln-list) (for-each prt-one soln-list))
		((string? soln-list) (display sln-list))
	)
)

; -----------------------------------------------------------------------
; Run all rules in a list of rules -- i.e. apply cog-bind to each.
;
(define (apply-all-rules rule-list)
	(for-each
		(lambda (rule) 
			(cog-delete ; need to delete the returned ListLink
				(cog-bind rule)
			)
		)
		rule-list
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

	; Hmm. Seems like sents is never null, unless there's a programming
	; error in RelEx.  Otherwise, it always returns something, even if the
	; input was non-sense. We'll check anyway, just to avoid crudola.
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

	; Attach parses to an anchor where they can be found.
	(attach-new-parses (get-new-parsed-sentences))

	; Apply a set of rules to determine if we have a truth query.
	(apply-all-rules *truth-query-id-list*)

	; If this is a truth query, then handle it
	(if (not (null? (get-truth-queries)))
		(let ()
			(display "Hello ")
			(display nick)
			(display ", you asked a truth-query question: ")
			(display txt)
			(newline)
			(chat-return "(say-try-svo-truth-query)")
		)
	)

	; Apply a set of rules to determine if we have a WH-question
	(apply-all-rules *wh-question-id-list*)

	(if (not (null? (get-wh-qvars)))
		(let ()
			(display "Hello ")
			(display nick)
			(display ", you asked a WH-question: ")
			(display txt)
			(newline)
			(chat-return "(say-try-svo-qa)")
		)
	)

	; If we are here, its not a question, but a statement
	(let ()
		(display "Hello ")
		(display nick)
		(display ", you made a statement: ")
		(display txt)
		(newline)
		(chat-return "(say-declaration)")
	)
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
	(find-truth-assertions)

(dbg-display "truth-assertions are:\n")
(display 
	(promote-to-seme same-modifiers-promoter (get-truth-assertions))
)
(end-dbg-display)

	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; *bottom-anchor* -- anchor triples for pattern matching.
;
; In order for the pattern matcher to locate things attached to
; anchors, they need to be attached in a way to "keep things simple"
; The simplest way to do this seems to be to attach one of the
; parts of the triple to the anchor.  e.g. given the triple
; "made_from(clay,pottery)" this attaches "clay" to the anchor.
;
; The attachment to this anchor is done by the implications in the 
; wh-question-id-list -- these implications find (identify) questions,
; and anchor them.
;
; The question-answering rules look for things on this anchor.
;
(define *bottom-anchor* (AnchorNode "# TRIPLE BOTTOM ANCHOR"))

(define (get-wh-qvars)
	(cog-chase-link 'ListLink 'WordInstanceNode *bottom-anchor*)
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

		; The cog-bind returns a ListLink. We need to nuke
		; that, as otherwise it will only cause trouble later.
		(let ((rslt (cog-bind quest)))
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
; say-try-svo-truth-query -- try answering a yes/no question
; Some questions are truth queries, posing a hypothesis, and asking for a 
; yes/no answer. This handles these.
;
(define (say-try-svo-truth-query)

	; First, we want to grab all of the words in the question, and
	; tag them with semes. (XXX We don't really need all the words,
	; only the words that are participating in relations.) Actually,
	; we pretty much just need to promote the verb, everything else
	; will follow.
	(for-each same-modifiers-promoter
		(concatenate! 
			(map parse-get-words
				(sent-list-get-parses (get-new-parsed-sentences))
			)
		)
	)
	
	; Try each truth-query template ... 
	(loop-over-questions *truth-query-rule-list*)

	; If there's no answer, try again, with triples processing
	; this time.
	(let ((ans (chat-get-simple-answer)))

		(if (null? ans)
			; No answer, try the prep-triples approach
			(chat-return "(say-try-triple-truth-query)")

			; The answer must be yes.
			(chat-prt-soln "SVO Truth query determined: Yes, verb was: " ans)
		)
	)

	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; say-try-triple-truth-query -- try answering a triples-based truth query
;
(define (say-try-triple-truth-query)

	; Run the triples processing.  
	(attach-sents-for-triple-processing (get-new-parsed-sentences))
	(create-triples)
	(dettach-sents-from-triple-anchor)

	; Try each truth-query template ... 
	; (loop-over-questions *truth-query-rule-list*)
(dbg-display "triple-truth-query trips are:\n")
(display (get-new-triples))
(end-dbg-display)
;	(let* ((trips (get-new-triples))
;		(trip-semes (promote-to-seme same-modifiers-promoter trips))
;		)
;(dbg-display "triple-truth-query semes are:\n")
;(display trip-semes)
;(end-dbg-display)
;	)

	; The meta-idea here is that we assume a "closed universe" -- 
	; If a truth-query question was asked, and we can't answer in the 
	; affirmative, then we will answer in the negative. This is not
	; correct in the open-universe model, where the answer would be
	; "no I don't really have enough information to answer this".
	(let ((ans (chat-get-simple-answer)))
		(if (null? ans)
			; No answer, tell them so.
			(chat-prt-soln "Triples truth query determined: No, not that I know of. " '())

			; The answer must be yes.
			(chat-prt-soln "Triples truth query determined: Yes, verb was: " ans)
		)
	)

	(chat-return "(say-final-cleanup)")
)

; -----------------------------------------------------------------------
; say-try-svo-qa -- try answering WH-questions using SVO pattern matching
; "SVO" is "Subject-Verb-Object", and the WH-question typically has a
; WH-word in the subject or object position. The answer to the question is
; the word that occupies the same location in another SVO assertion.
;
(define (say-try-svo-qa)

	; First, we want to grab all of the words in the question, and
	; tag them with semes. (XXX We don't really need all the words,
	; only the words that are participating in relations.) Actually,
	; we pretty much just need to promote the verb, everything else
	; will follow. Err.. well, and preps too ... 
	(for-each same-modifiers-promoter
		(concatenate! 
			(map parse-get-words
				(sent-list-get-parses (get-new-parsed-sentences))
			)
		)
	)
	
	; Try each wh-question template ... 
	(loop-over-questions *wh-question-rule-list*)

	; If SVO pattern-matching found an answer, print it.
	(let ((ans (chat-get-simple-answer)))
		(if (not (null? ans))
			(let () 
				(chat-prt-soln "SVO pattern match found: " ans)
				(chat-return "(say-final-cleanup)")
			)
		)
	)

	(chat-return "(say-try-triple-qa)")
)

; -----------------------------------------------------------------------
; say-try-triple-qa -- try answering questions using triples.
; Takes a question, tries to extract prepositional triples
; from it, then attempts a pattern match on the results.
;
(define (say-try-triple-qa)

	; Run the triples processing.  
	(attach-sents-for-triple-processing (get-new-parsed-sentences))
	(create-triples)
	(dettach-sents-from-triple-anchor)

   ; Perform pattern matching on the triples.
	(let ((trips (get-new-triples)))
		(if (null? trips)
			(let ()
				(dbg-display "No question triples found, try deduction.\n")
				(end-dbg-display)
				(chat-return "(say-try-deduction)")
			)
		)

		; Pull in any semes that might be related...
		(fetch-related-semes trips)

(dbg-display "duude question trips are:\n")
(display trips)
(end-dbg-display)

		; Now try to find answers to the question
		(loop-over-questions *wh-trip-question-rule-list*)

		(let ((ans (chat-get-simple-answer)))

			(if (null? ans)
				(let ()
					(dbg-display "No answer triples found, try deduction.\n")
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
					(ans (cog-bind (make-simple-chain ftrip)))
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

	; Delete  the list of solutions, so that we don't accidentally
	; replay it when the next question is asked.
	(chat-release-simple-answer)

	; cleanup -- these sentences are not new any more
	(release-new-parsed-sents)
	""
)

; -----------------------------------------------------------------------
