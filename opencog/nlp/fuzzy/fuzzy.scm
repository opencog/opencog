
(define-module (opencog nlp fuzzy))

(use-modules (srfi srfi-1)
             (opencog)
             (opencog query)  ; for cog-fuzzy-match
             (opencog nlp)
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
  howevr, they could be from other sources.
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
    (cog-fuzzy-match r2l-set 'SetLink exclude-list)
)

; ----------------------------------------------------------
(define-public (get-answers sent-node)
"
  Find answers (i.e., similar sentences that share some keyword) from
  the Atomspace by using the fuzzy pattern matcher. By default, it
  excludes sentences with TruthQuerySpeechAct and InterrogativeSpeechAct.

  Accepts a SentenceNode as the input.
  Returns one or more sentence strings -- the answers.

  For example:
     (get-answers (car (nlp-parse \"What did Pete eat?\")))
  OR:
     (get-answers (SentenceNode \"sentence@123\"))

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

    (let* ( ; List of setence types to not consider
            (exclude-list
                (list (DefinedLinguisticConceptNode "TruthQuerySpeechAct")
                      (DefinedLinguisticConceptNode "InterrogativeSpeechAct")))

            ; r2l-set is the ... r2l-set of the SentenceNode
            (r2l-set (get-r2l-set-of-sent sent-node))

            ; fzset is the set of similar r2l-sets.
            (fzset (cog-fuzzy-match r2l-set 'SetLink exclude-list)

            ; reply is a list of the reply sentences,
            ; generated by  Microplanner and SuReal
            (reply (gen-sentences (cog-outgoing-set fzset)))
        )
(trace-msg "get-answers generated reply is ")
(trace-msg reply)

        ; Delete identical sentences from the return set
        (delete-duplicates reply)
    )
)
