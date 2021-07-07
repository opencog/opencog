(define-module (opencog nlp fuzzy))

(use-modules (opencog oc-config))
(load-extension (string-append opencog-ext-path-nlp-fuzzy "libnlpfz") "opencog_nlp_fuzzy_init")

(use-modules (srfi srfi-1)
             (ice-9 optargs)      ; for doing define*-public
             (opencog)
             (opencog nlp)
             (opencog nlp oc)
             (opencog nlp sureal)
             (opencog nlp microplanning))

; ----------------------------------------------------------

(define-public (get-r2l-set-of-sent sent-node)
"
  get-r2l-set SENT-NODE - Return the collection of R2L expressions
  for the SentenceNode SENT-NODE.  This just a simple pointer-chase.

  XXX FIXME This is broken, because it assumes that there is only
  one parse, and one valid interpretation for that parse.
  In fact, there may be many parses, of varying quality,
  and there may be multiple interpretations, too.  The best
  ones are the ones with the highest TV (and confidence).

  Eventually, this routine should be replaced by one that does
  proper parse-ranking and confidence-ranking.
"
    (let* ( ; parse is the ParseNode for the sentence
            (parse (car (cog-chase-link 'ParseLink 'ParseNode sent-node)))

            ; intrp is the InterpretationNode for the sentence
            (intrp (car (cog-chase-link 'InterpretationLink 'InterpretationNode parse)))
        )
        ; Return the R2L SetLink of the input sentence
        (car (cog-chase-link 'ReferenceLink 'SetLink intrp))
    )
)

; ----------------------------------------------------------

(define-public (gen-sentences setlinks)
"
  Generate sentences from each of the R2L-SetLinks

  Typically, the SetLinks are those found by the fuzzy matcher;
  however, they could be from other sources.
  TODO: May need to filter out some of the contents of the SetLinks
  before sending each of them to Microplanner
"

    ; Find the speech act from the SetLink and use it for Microplanning
    ; XXX FIXME the Microplanner should use the same speech-act types as
    ; everyone else, so that we don't have to do this horrific string
    ; manginling.
    (define (get-speech-act setlink)
        (define speech-act-node-name
            (filter (lambda (name)
                (if (string-suffix? "SpeechAct" name) #t #f))
                     (map cog-name (cog-filter 'DefinedLinguisticConceptNode (cog-get-all-nodes setlink)))))

        ; If no speech act was found, return "declarative" as default
        (if (> (length speech-act-node-name) 0)
            (string-downcase (substring (car speech-act-node-name) 0 (string-contains (car speech-act-node-name) "SpeechAct")))
            "declarative"
        )
    )

    (append-map (lambda (r)
        ; Send each of the SetLinks to Microplanner to see if
        ; they are good
        (let* ( (spe-act (get-speech-act r))
                (seq-and (AndLink (cog-outgoing-set r)))
                (m-results (microplanning seq-and spe-act
                     *default_chunks_option* #f)))
(trace-msg "gen-sentences spe-act is ")
(trace-msg spe-act)
(trace-msg "\ngen-sentences m-results are ")
(trace-msg m-results)
(trace-msg "\n")
            ; Don't send it to SuReal in case it's not good
            ; (i.e. Microplanner returns #f)
            (if m-results
                (append-map
                    ; Send each of the SetLinks returned by
                    ; Microplanning to SuReal for sentence generation
                    (lambda (m) (sureal (car m)))
                    m-results
                )
                '()
            )
        ))
        setlinks
    )
)

; ----------------------------------------------------------

(define-public (fuzzy-match-sent sent-node exclude-list)
"
  fuzzy-match-sent - Perform fuzzy-match to find sentences that
  are similar to SENT-NODE.  Exclude EXCLUDE-LIST sentence types
  from the search.

  Returns one or more R2L-SetLinks containing sentence forms
  that are similar to the input sentence.

  This is just a thin wrapper that locates the R2L-SetLink for
  the given sentence.

  XXX FIXME This is broken, because it assumes that there is only
  one parse, and one valid interpretation for that parse.
  In fact, there may be many parses, of varying quality,
  and there may be multiple interpretations, too.  The best
  ones are the ones with the highest TV (and confidence).

  Eventually, this routine should be replaced by one that does
  proper parse-ranking and confidence-ranking.
"
    ; Return the set of similar sets.
    (define r2l-set (get-r2l-set-of-sent sent-node))
    (nlp-fuzzy-match r2l-set 'SetLink exclude-list #f)
)

; ----------------------------------------------------------
(define*-public (get-fuzzy-answers sent-node #:key (do-microplanning #t))
"
  Find answers (i.e., similar sentences that share some keyword) from
  the Atomspace by using the fuzzy pattern matcher. By default, it
  excludes sentences with TruthQuerySpeechAct and InterrogativeSpeechAct.

  Accepts a SentenceNode as the input.
  Returns one or more sentence strings -- the answers.

  For example:
     (get-fuzzy-answers (car (nlp-parse \"What did Pete eat?\")))
  OR:
     (get-fuzzy-answers (SentenceNode \"sentence@123\"))

  Possible result:
     (Pete ate apples .)

  XXX FIXME This is broken, because it assumes that there is only
  one parse, and one valid interpretation for that parse.
  In fact, there may be many parses, of varying quality,
  and there may be multiple interpretations, too.  The best
  ones are the ones with the highest TV (and confidence).

  Eventually, this routine should be replaced by one that does
  proper parse-ranking and confidence-ranking.
"

    ; Post processing for the results found by the fuzzy-matchers. May
    ; be generally useful, if we have multiple results and we want to
    ; merge or ignore some of them.
    ; TODO: Subject to change, currently it returns the top ones that
    ; have the same similarity score.
    (define (post-process fset)
        (let ( (max-score 0)
               (results '()))
            (for-each (lambda (s)
                (let ( (score (cog-number (cadr (cog-outgoing-set s)))))
                    ; Make sure it can be used to generate a sentence by sureal
                    (if (and (>= score max-score) (not (equal? (sureal (car (cog-outgoing-set s))) '())))
                        (begin
                            (set! results (append results (list (car (cog-outgoing-set s)))))
                            (set! max-score score))
                        #f)))
            (cog-outgoing-set fset))
            (cog-extract! fset)
            results))

    (let* ( ; List of setence types to not consider
            (exclude-list
                (list (DefinedLinguisticConceptNode "TruthQuerySpeechAct")
                      (DefinedLinguisticConceptNode "InterrogativeSpeechAct")))

            ; r2l-set is the ... r2l-set of the SentenceNode
            (r2l-set (get-r2l-set-of-sent sent-node))

            ; fzset is the set of similar r2l-sets.
            (fzset (nlp-fuzzy-match r2l-set 'SetLink exclude-list #f))

            ; ppset is a set of atoms that will be used for sentence generation
            ; after doing some post-processing
            (ppset (post-process fzset))
        )

        ; Return the results as a list of sentences (strings)
        (if do-microplanning
            (map string-join (delete-duplicates (gen-sentences ppset)))
            (delete-duplicates (map
                (lambda (s)
                    (let ((result (sureal s)))
                        ; SuReal returns lists of words when calling `create-sentence`
                        ; but a string when calling `get-sentence`
                        (if (list? result)
                            (string-join (car result))
                            result
                        )
                    )
                )
                ppset
            ))
        )
    )
)
