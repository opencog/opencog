; Action APIs and schemas

; --------------------------------------------------------------
; APIs for forming GroundedSchemas that are use for exectuing
; actions.
; NOTE: For testing use (opencog eva-behavior) module. For
; running use (opencog movement) module. This is because the
; APIs are atomese DefinedPredicates.
; TODO: List out the DefinedPredicates that are used as API, so as to
; use delete-definition. Also adapt the scheme function naming convention
; to make remembering easier.
; --------------------------------------------------------------
(define fini (Node "finished-action"))

(DefineLink
  (DefinedSchema "say")
  (LambdaLink
    (VariableList
      (Variable "sentence")
      (Variable "fallback-id"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "say")
        (Variable "sentence")))
  )
)

(define (fallback_on fallback-id)
"
  fallback_on  FALLBACK-ID

  Use the fallback system identified by FALLBACK-ID
"
  (cog-execute! (Put (DefinedSchema "say") (List (Concept "") fallback-id)))
  fini
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "say-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
       (List (Concept "shutup")))
  )
)

(define (shutup)
  (cog-execute! (Put (DefinedSchema "say-cancel") (List)))
  fini
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gaze-at")
  (LambdaLink
    (VariableList
      (Variable "face-id")
      (Variable "speed"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "gaze-at")
        (Variable "face-id")
        (Variable "speed")))
  )
)

(define (look face-id speed)
  (cog-execute! (Put (DefinedSchema "gaze-at") (List face-id speed)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gaze-at-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "gaze-at-cancel")))
  )
)

(define (look_cancel)
  (cog-execute! (Put (DefinedSchema "gaze-at-cancel") (List)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "blink")
  (LambdaLink
    (VariableList
      (Variable "mean")
      (Variable "variation"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "blink")
        (Variable "mean")
        (Variable "variation")))
  )
)

(define (blink mean variation)
  (cog-execute! (Put (DefinedSchema "blink") (List mean variation)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "blink-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "blink-cancel")))
  )
)

(define (blink_cancel)
  (cog-execute! (Put (DefinedSchema "blink-cancel") (List)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "emote")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "magnitude")
      (Variable "duration")
      (Variable "blend"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "emote")
        (Variable "name")
        (Variable "magnitude")
        (Variable "duration")
        (Variable "blend")))
  )
)

(define (emote name magni duration blend)
  (cog-execute!
    (Put (DefinedSchema "emote") (List name magni duration blend)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "gesture")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "speed")
      (Variable "magnitude")
      (Variable "repeat"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "gesture")
        (Variable "name")
        (Variable "speed")
        (Variable "magnitude")
        (Variable "repeat")))
  )
)

(define (gesture name speed magni repeat)
  (cog-execute!
    (Put (DefinedSchema "gesture") (List name speed magni repeat)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "soma")
  (LambdaLink
    (VariableList
      (Variable "name")
      (Variable "magnitude")
      (Variable "rate")
      (Variable "ease_in"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "soma")
        (Variable "name")
        (Variable "magnitude")
        (Variable "rate")
        (Variable "ease_in")))
  )
)

(define (soma name magni rate ease-in)
  (cog-execute!
    (Put (DefinedSchema "soma") (List name magni rate ease-in)))
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "soma-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "soma-cancel")))
  )
)

(define (soma_cancel)
  (cog-execute! (Put (DefinedSchema "soma-cancel") (List)))
)

; --------------------------------------------------------------
(DefineLink
  (DefinedSchema "saccade")
  (LambdaLink
    (VariableList
      (Variable "mean")
      (Variable "variation")
      (Variable "paint_scale")
      (Variable "eye_size")
      (Variable "eye_distance")
      (Variable "mouth_width")
      (Variable "mouth_height")
      (Variable "weight_eyes")
      (Variable "weight_mouth"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "saccade")
        (Variable "mean")
        (Variable "variation")
        (Variable "paint_scale")
        (Variable "eye_size")
        (Variable "eye_distance")
        (Variable "mouth_width")
        (Variable "mouth_height")
        (Variable "weight_eyes")
        (Variable "weight_mouth")))
  )
)

(define (saccade-args . nums)
  (List (map (lambda (x) (Number x)) nums ))
)

(define (saccade_explore)
  (cog-execute! (Put (DefinedSchema "saccade")
    (saccade-args 0.8 0.3 1.0 15.0 100.0 90.0 27.0 0.8 0.2))
  )
)

(define (saccade_listen)
  (cog-execute! (Put (DefinedSchema "saccade")
    (saccade-args 1.0 0.6 0.3 11.0 80.0 50.0 13.0 0.8 0.2))
  )
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "saccade-cancel")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "saccade-cancel")))
  )
)

(define (saccade_cancel)
  (cog-execute! (Put (DefinedSchema "saccade-cancel") (List)))
)

; --------------------------------------------------------------
(define* (start_timer #:optional (timer-id (Concept "Default-Timer")))
"
  start_timer TIMER-ID (optional)

  Record the current time for TIMER-ID.
  If TIMER-ID is not given, a default timer will be used.
"
  (set-time-perceived! (Concept (cog-name timer-id))
    (FloatValue (current-time-us)))
  fini
)

(define (decrease_urge goal value)
"
  decrease_urge GOAL VALUE

  Decrease the urge of GOAL by VALUE.
"
  (psi-decrease-urge (Concept (cog-name goal)) (string->number (cog-name value)))
  fini
)

(define (increase_urge goal value)
"
  increase_urge GOAL VALUE

  Increase the urge of GOAL by VALUE.
"
  (define goal-node (Concept (cog-name goal)))
  (define related-psi-rules
    (filter psi-rule? (cog-incoming-set goal-node)))

  (psi-increase-urge goal-node (string->number (cog-name value)))

  ; Stimulate the rules associate with this goal
  (for-each
    (lambda (r)
      (cog-stimulate r default-stimulus))
    related-psi-rules)

  fini
)

; --------------------------------------------------------------
(define ddg-src (def-source "DuckDuckGo" "ask-duckduckgo"))

(define (ask-duckduckgo sent)
  (define query (source-query sent))
  ; A very crude and limited way to get the first sentence
  ; from the respond, but is OK in this context
  ; TODO: Replace by an actual sentence splitter
  (define (get-first-sentence str)
    (define default-length 50)
    (if (< (string-length str) default-length)
      (substring str 0 (string-index str #\.))
      (substring str 0 (+ default-length
        (string-index (substring str default-length) #\.)))
    )
  )

  (let* ((url (string-append "http://api.duckduckgo.com/?q="
           (string-downcase query) "&format=xml"))
         (body (xml->sxml (response-body-port (http-get url #:streaming? #t))))
          (resp (car (last-pair body)))
         (abstract (find (lambda (i)
           (and (pair? i) (equal? 'Abstract (car i)))) resp)))

    (if (equal? (length abstract) 1)
      (cog-logger-debug schema-logger "No answer found from DuckDuckGo!")
      (let* ((ans (car (cdr abstract)))
             (ans-1st-sent (get-first-sentence ans)))
        (source-set-result! sent ddg-src ans-1st-sent))
    )
  )
)

; --------------------------------------------------------------
(define wa-src (def-source "Wolfram|Alpha" "ask-wolframalpha"))

(define wa-appid "")
(define (set-wa-appid! id)
"
  set-wa-appid ID

  Set the Wolfram|Alpha AppID to ID, which is needed for using the Wolfram|Alpha APIs.
"
  (set! wa-appid id))

(define (ask-wolframalpha sent)
  (define query (source-query sent))
  (if (string-null? wa-appid)
    (cog-logger-debug schema-logger "AppID for Walfram|Alpha has not been set yet!")
    (let* ((query-no-spaces
             (regexp-substitute/global #f " " (string-downcase query) 'pre "+" 'post))
           (url (string-append
             "http://api.wolframalpha.com/v2/query?appid="
               wa-appid "&input=" query-no-spaces "&format=plaintext"))
           (body (xml->sxml (response-body-port (http-get url #:streaming? #t))))
           (resp (car (last-pair body)))
           (ans
             (find
               (lambda (i)
                 (and (pair? i)
                      (equal? 'pod (car i))
                      (equal? 'title (car (cadr (cadr i))))
                      ; The tags we are looking for
                      (not (equal? (member (cadr (cadr (cadr i)))
                        (list "Result" "Definition" "Definitions"
                              "Basic definition" "Basic information")) #f))))
               resp)))

      (if (equal? ans #f)
        (cog-logger-debug schema-logger "No answer found from Wolfram|Alpha!")
        (let* ((text-ans (cadr (cadddr (cadddr ans))))
               ; Remove '(', ')', and '|' from the answer, if any
               (cleaned-ans (string-trim (string-filter
                   (lambda (c) (not (or (char=? #\( c)
                                        (char=? #\) c)
                                        (char=? #\| c)))) text-ans)))
               ; Remove newline and split them into words
               (ans-1st-line (car (string-split cleaned-ans #\newline))))
          (source-set-result! sent wa-src ans-1st-line)
        )
      )
    )
  )
)

(load-trail-3)
;-------------------------------------------------------------------------------
(define pln-src (def-source "pln" "ask-pln"))

(define* (ask-pln sent #:optional (steps 24))
  ; TODO: use query for filtering results, by using similarity measures b/n
  ; the query and the inferred outputs. There may be multiple layers of
  ; filters.
  (define query (source-query sent))
  ; TODO: How to choose an appropriate trail or set of trails?
  (update-inferences rb-trail-3 steps (pln-get-recorded-time))
  (source-set-result! sent pln-src (pln->sureal rb-trail-3))
)

(define (pln_start_recording)
"
  pln_start-recording

  Records the present time for usage in getting nlp inputs for inferrence.
"
  (pln-record-current-time)
  fini
)

; --------------------------------------------------------------
(define (stimulate_words . words)
"
  stimulate_words WORDS

  Stimulate the WordNodes corresponding to WORDS.
"
  (for-each
    (lambda (w) (cog-stimulate (Word (cog-name w)) 200))
    words)
  fini
)

(define (stimulate_concepts . concepts)
"
  stimulate_concepts CONCEPTS

  Stimulate the ConceptNodes corresponding to CONCEPTS.
"
  (for-each
    (lambda (c) (cog-stimulate (Concept (cog-name c)) 200))
    concepts)
  fini
)

(define (stimulate_rules . rule-labels)
"
  stimulate_rules RULE-LABELS

  Stimulate the rules with RULE-LABELS.
"
  (for-each
    (lambda (r) (cog-stimulate (get-rule-from-alias (cog-name r)) 200))
    rule-labels)
  fini
)

(define (set_rule_sti rule-label val)
"
  set_rule_sti RULE-LABEL VAL

  Set the STI of a rule with RULE-LABEL to VAL.
"
  (cog-set-sti!
    (get-rule-from-alias (cog-name rule-label))
    (string->number (cog-name val)))
)

(define (max_sti_words . words)
"
  max_sti_words WORDS

  Maximize the STI of the WordNodes correspondings to WORDS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (w) (cog-set-sti! (Word (cog-name w)) max-sti))
    words)
  fini
)

(define (max_sti_concepts . concepts)
"
  max_sti_concepts . CONCEPTS

  Maximize the STI of the ConceptNodes corresponding to CONCEPTS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (c) (cog-set-sti! (Concept (cog-name c)) max-sti))
    concepts)
  fini
)

(define (max_sti_rules . rule-labels)
"
  max_sti_rules . RULE-LABELS

  Maximize the STI of the rules with RULE-LABELS.
"
  (define max-sti (cog-av-sti (car (cog-af 1))))
  (for-each
    (lambda (r) (cog-set-sti! (get-rule-from-alias (cog-name r)) max-sti))
    rule-labels)
  fini
)

(define (min_sti_words . words)
"
  min_sti_words WORDS

  Minimize the STI of the WordNodes correspondings to WORDS.
"
  (for-each
    (lambda (w) (cog-set-sti! (Word (cog-name w)) 0))
    words)
  fini
)

(define (min_sti_concepts . concepts)
"
  min_sti_concepts . CONCEPTS

  Minimize the STI of the ConceptNodes corresponding to CONCEPTS.
"
  (for-each
    (lambda (c) (cog-set-sti! (Concept (cog-name c)) 0))
    concepts)
  fini
)

(define (min_sti_rules . rule-labels)
"
  min_sti_rules . RULE-LABELS

  Minimize the STI of the rules with RULE-LABELS.
"
  (for-each
    (lambda (r) (cog-set-sti! (get-rule-from-alias (cog-name r)) 0))
    rule-labels)
  fini
)

; --------------------------------------------------------------
(define* (send_query query #:optional source)
"
  send_query QUERY [SOURCE]

  Calls the interface of SOURCE with name of the node or the concatenation
  of the names of the Nodes in a ListLink. If SOURCE is not specified then
  the query is passed to all the sources.

  It also signals that processing has started.
"
  (define sent (ghost-get-curr-sent))
  (define query-str
    (if (equal? 'ListLink (cog-type query))
      (string-join (map cog-name (cog-outgoing-set query)) " ")
      (cog-name query)
    ))

  (define (spawn-source src)
    (call-with-new-thread (lambda ()
      (cog-set-value! src source-latest-sent-key sent)
      (source-set-processing! src sent (stv 1 1))
      (eval-string (format #f "(~a ~a)" (source-func-name src) sent)))))

  (source-set-query! sent query-str)

  (if source
    (spawn-source (Concept (cog-name source)))
    (par-map spawn-source (get-sources))
  )
  fini
)

(define* (get_answer #:optional source)
"
  get_answer [SOURCE]

  Get the latest answer from SOURCE. If SOURCE is not specified then the
  latest answer is randomly selected from one of the sources.
"
  (define src '())
  (define sent '())
  (if source
    (begin
      (set! src (Concept (cog-name source)))
      (set! sent (cog-value src source-latest-sent-key))
      (set! answer-src src))
    ; Since the assumption is that there are only ordered goals, any source
    ; will work.
    (begin
      (set! sent (cog-value (car (get-sources)) source-latest-sent-key))
      (set! answer-src (cog-value sent (Predicate "random-source"))))
  )

  (cond
    ((nil? sent) fini) ; If run before query is sent.
    (source (Concept (cog-value-ref (source-result sent src) 0)))
    (else (Concept (cog-value-ref
      (source-result sent (cog-value sent (Predicate "random-source"))) 0)))
  )
)

(define (get_answer_source)
"
  get_answer_source

  Get the source of the most recent answer returned by the 'get_answer' schema.
"
  answer-src
)

; --------------------------------------------------------------
(define (send_stochastic_question)
"
  send_stochastic_question

  Send the current input sentence to the stochastic question
  generator to see if it's possible to generate a followup
  question for it.
"
  (define output '())

  ; To generate a question based on input sentence
  (define (s2q bindlk)
    ; See if the BindLink works
    (define bind-results (cog-outgoing-set (cog-execute! bindlk)))

    ; Send the BindLink results to SuReal for sentence generation
    (define sureal-results
      (if (nil? bind-results)
        (list)
        (append-map sureal bind-results)))

    ; Select one of the sentences generated, just in case there are
    ; more than one
    (define selected-result
      (if (nil? sureal-results)
        (list)
        (car sureal-results)))

    ; Hacky-ugly post-processing to turn e.g. "you" -> "I" etc
    (define post-proc-result
      (map
        (lambda (w)
          (cond
            ((string-ci=? "I" w) "you")
            ((string-ci=? "you" w) "I")
            (else w)))
        selected-result))

    ; (format #t "Bind:\n~a\n" bind-results)
    ; (format #t "SuReal:\n~a\n" sureal-results)
    ; (format #t "Selected:\n~a\n" selected-result)
    ; (format #t "Post-processed:\n~a\n" post-proc-result)

    ; Record the output
    (set! output (string-join post-proc-result))

    ; Return
    (not (string-null? output)))

  (define (record-sq-output)
    (cog-set-value!
      (ghost-get-curr-sent)
      (Concept "StochasticQuestion")
      (StringValue output)))

  ; Try from the most specific one to the least
  (cond
    ((s2q bind-subj-obj-det-nn) (record-sq-output))
    ((s2q bind-subj-obj-det) (record-sq-output))
    ((s2q bind-subj-obj-nn) (record-sq-output))
    ((s2q bind-subj-obj) (record-sq-output))
    ((s2q bind-subj) (record-sq-output)))
)

(define (get_stochastic_question)
"
  get_stochastic_question

  Return the stochastic question generated for the current input.
"
  (Concept (cog-value-ref
    (cog-value (ghost-get-curr-sent) (Concept "StochasticQuestion")) 0))
)

; --------------------------------------------------------------
(define (get_neck_dir)
"
  get_neck_dir

  Get the direction of the head turned.
"
  (define directions
    (cog-outgoing-set
      (cog-execute!
        (Get
          (TypedVariable
            (Variable "$x")
            (Signature
              (Evaluation
                (Predicate "looking")
                (List (Concept "I") (Type "ConceptNode")))))
          (Variable "$x"))
      )
    )
  )

  (gddr
    (fold
      (lambda (x rtn)
        (cond ((nil? rtn) x)
              ((> (time-perceived x) (time-perceived rtn)) x)
              (else rtn)))
      (list)
      directions
    )
  )
)

; --------------------------------------------------------------
(Define
  (DefinedSchema "sing")
  (Lambda
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List (Concept "sing")))
  )
)

(define (sing)
  (cog-execute! (Put (DefinedSchema "sing") (List)))
)

; --------------------------------------------------------------
(define-syntax define-emotion-parameter (lambda (x)
  "define-emotion-parameter EMO-PARAM

   Defines get_EMO-PARAM, increase_EMO-PARAM, decrease_EMO-PARAM,
   and neutralize_EMO-PARAM functions, were EMO-PARAM is the name of
   an openpsi parameter.
  "

  (define (new-id prefix id)
  "prefix = a string
   id = a pattern variable(aka keyword, template-identifier).
   See https://scheme.com/tspl4/syntax.html#./syntax:h4 for details.
  "
    (datum->syntax id (string->symbol
      (string-append prefix (syntax->datum id)))))

  (syntax-case x ()
    ((_ emo-param)
     (with-syntax ((get (new-id "get_" #'emo-param))
                   (inc (new-id "increase_" #'emo-param))
                   (dec (new-id "decrease_" #'emo-param))
                   (neu (new-id "neutralize_" #'emo-param)))
         #'(begin
             (define (get) (Concept (number->string
                             (psi-param-value (psi-param emo-param)))))
             (define (inc num) (psi-param-increase! (psi-param emo-param)
                                 (string->number (cog-name num))))
             (define (dec num) (psi-param-decrease! (psi-param emo-param)
                                 (string->number (cog-name num))))
             (define (neu num) (psi-param-neutralize! (psi-param emo-param)
                                 (string->number (cog-name num)))))
     ))))
)

(define-emotion-parameter "valence")
(define-emotion-parameter "arousal")

; --------------------------------------------------------------
(Define
  (DefinedSchema "set-parameter")
  (LambdaLink
    (VariableList
      (Variable "component")
      (Variable "parameter")
      (Variable "value"))
    (ExecutionOutput
      (GroundedSchema "scm: print-by-action-logger")
      (List
        (Concept "set-parameter")
        (Variable "component")
        (Variable "parameter")
        (Variable "value")))
  )
)

(define (set-parameter component parameter value)
  (cog-execute!
    (Put
      (DefinedSchema "set-parameter")
      (List component parameter value)))
)

; --------------------------------------------------------------
; The *voice* schemas follow from ssml were 'speed' == 'rate'
; See https://www.w3.org/TR/speech-synthesis/#edef_prosody
;
; TODO: When there is an api to get information about the range of volume
; and speeds that is possible to use.
(define (increase_voice_speed percent)
  (set-parameter (Concept "speech") (Concept "rate")
    (Number (+ 1.0 (/ (string->number (cog-name percent)) 100))))
)

(define (decrease_voice_speed percent)
  (define percent-num (string->number (cog-name percent)))
  (if (<= 100.0 percent-num)
    (error "Speed can't be decreased by 100% or more."))

  (set-parameter (Concept "speech") (Concept "rate")
    (Number (- 1.0 (/ percent-num 100))))
)

(define (increase_voice_volume percent)
  (set-parameter (Concept "speech") (Concept "volume")
    (Number (* 20 (log10 (+ 1.0 (/ (string->number (cog-name percent)) 100)))))
  )
)

(define (decrease_voice_volume percent)
  (define percent-num (string->number (cog-name percent)))
  (if (<= 100.0 percent-num)
    (error "Volume can't be decreased by 100% or more."))

  (set-parameter (Concept "speech") (Concept "volume")
    (Number (* 20 (log10 (- 1.0 (/ percent-num 100)))))
  )
)
