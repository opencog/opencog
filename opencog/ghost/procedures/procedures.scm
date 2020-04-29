(define-module (opencog ghost procedures)
  #:use-module (ice-9 optargs)
  #:use-module (ice-9 regex)
  #:use-module (ice-9 threads)
  #:use-module (ice-9 eval-string)
  #:use-module (srfi srfi-1)
  #:use-module (sxml simple)
  #:use-module (web client)
  #:use-module (web response)
  #:use-module (opencog)
  #:use-module (opencog attention-bank)
  #:use-module (opencog logger)
  #:use-module (opencog exec)
  #:use-module (opencog nlp)
  #:use-module (opencog nlp chatbot)
  #:use-module (opencog nlp sureal)
  #:use-module (opencog openpsi)
  #:use-module (opencog pln)
  #:use-module (opencog ure)
  #:use-module (opencog ghost)
  #:export (
    ; -------------------- Perception -------------------- ;
    ; Perception switches
    perception-start!
    perception-stop!

    ; Perception input apis
    perceive-face
    perceive-emotion
    perceive-word
    perceive-face-talking
    perceive-eye-state
    perceive-neck-dir

    ; Sensory input hooks
    perceive-word-hook

    ; -------------------- Predicates -------------------- ;
    ; Perceptual predicates
    is_word_perceived

    ; Perceptual predicates for a talking face
    is_talking
    is_not_talking

    ; Perceptual predicates for visibility of a face
    is_face_perceived

    ; Perceptual predicates for emotion of a face
    is_emotion

    ; Time related predicates
    is_after_min

    ; Source predicates
    is_answer_received

    ; self-model
    is_neck_direction

    ; -------------------- Schemas -------------------- ;
    animation
    expression
    start_timer
    decrease_urge
    increase_urge
    stimulate_words
    stimulate_concepts
    stimulate_rules
    set_rule_sti
    max_sti_words
    max_sti_concepts
    max_sti_rules
    min_sti_words
    min_sti_concepts
    min_sti_rules
    print-by-action-logger
    fallback_on
    shutup
    look
    look_cancel
    blink
    blink_cancel
    emote
    gesture
    soma
    soma_cancel
    saccade_explore
    saccade_listen
    saccade_cancel
    sing
    get_neck_dir
    get_valence
    increase_valence
    decrease_valence
    neutralize_valence
    get_arousal
    increase_arousal
    decrease_arousal
    neutralize_arousal
    increase_voice_speed
    decrease_voice_speed
    increase_voice_volume
    decrease_voice_volume

    ; Source schemas
    send_query
    get_answer
    get_answer_source
    pln_start_recording

    ; Source interfaces
    ; This need to public so to be callable by send_query
    ask-duckduckgo
    ask-wolframalpha
    ask-pln

    ; Stochastic Question
    send_stochastic_question
    any_stochastic_question
    get_stochastic_question

    ; -------------------- Utilities -------------------- ;
    set-dti!
    get-dti
    is-model-true?
    any-model-true?
    set-time-perceived! ; temporarily exported
    time-perceived ; temporarily exported
    was-perceived?
    ghost-stimulate-timer
    action-logger
    percep-refractory-period
    set-percep-refractory-period!
    set-wa-appid!
  )
)

; --------------------------------------------------------------
; APIs used to create the atoms used to represent the world, aka
; world-model. These are not exported.
; --------------------------------------------------------------
; Identifier used for an unidentified source of perceptual stimulus.
(define any-id "")
; Node used to represent an unidentified source of perceptual stimulus.
(define any-node (Concept ""))

(define (get-models sign)
"
  get-models SIGN

  Returns a list containing all the atoms that have the pattern defined by
  the SignatureLink SIGN.
"
  (cog-outgoing-set (cog-execute!
    (Get
      (TypedVariable
        (Variable "model")
        sign)
      (Variable "model"))
  ))
)

; --------------------------------------------------------------
(define see-face-predicate (Predicate "see"))

(define see-face-sign
  (Signature
    (Evaluation
      see-face-predicate
      (List
        (Concept "I")
        (Type "ConceptNode"))))
)

(define (see-face face-id)
"
  Define the atom used to represent that the face represented by FACE-ID
  is being seen.
"
  (Evaluation
    see-face-predicate
    (List
      (Concept "I")
      (Concept face-id)))
)

; --------------------------------------------------------------
(define face-emotion-predicate (Predicate "is"))

(define face-emotion-sign
  (Signature
    (Evaluation
      face-emotion-predicate
      (List
        (Type "ConceptNode")
        (Type "ConceptNode"))))
)

(define (face-emotion face-id emotion-type)
"
  Define the atom used to represent EMOTION-TYPE of the face with
  id FACE-ID.
"
  (Evaluation
    face-emotion-predicate
    (List
      (Concept face-id)
      (Concept emotion-type)))
)

; --------------------------------------------------------------
(define face-talking-predicate (Predicate "talking"))

(define face-talking-sign
  (Signature
    (Evaluation
      face-talking-predicate
      (List
        (Type "ConceptNode"))))
)

(define (face-talking face-id)
"
  Define the atom used to represent that the face represented by FACE-ID
  is talking.
"
  (Evaluation
     face-talking-predicate
    (List
      (Concept face-id)))
)

; --------------------------------------------------------------
(define (eye-open face-id eye-id)
"
  eye-open FACE-ID EYE-ID

  Define the atom used to represent whether FACE-ID face's EYE-ID eye is open.
"
  (Evaluation
    (Predicate "eye-open")
    (List
      (Concept face-id)
      (Concept eye-id)))
)

; --------------------------------------------------------------
(define (looking dir)
"
  looking DIR

  Define the atom used to represent the direction of the neck turned,
  to the left/right.
"
  (Evaluation
    (Predicate "looking")
    (List
      (Concept "I")
      (Concept dir)))
)

; --------------------------------------------------------------
; APIs for inputing sensory information.
; --------------------------------------------------------------
;(define (perceived-face face-id x y z)
;  (cog-pointmem-map-atom facemap (Concept face-id)
;    (List (Number x) (Number y) (Number z)))
(define default-stimulus 150)


(define percep #f)
(define (perception-start!)
"
  perception-start!

  This declares that all perception apis should start recording changes
  when called.
"
  (set! percep #t)
)

(define (perception-stop!)
"
  perception-stop!

  This declares that all perception apis should stop recording changes
  when called.
"
  (set! percep #f)
)

(define (record-perception model new-conf)
  (let ((old-conf (cog-confidence model))
    (time (FloatValue (current-time-us))))

    (if percep (begin
      (set-time-perceived! model (FloatValue (current-time-us)))
      (set-event-times! model old-conf new-conf time)
      (cog-set-tv! model (stv 1 new-conf))
      (cog-stimulate model default-stimulus)))

    model
  )
)

(define (perceive-face face-id confidence)
"
  perceive-face FACE-ID CONFIDENCE

  Returns the atom representing whether the face with id FACE-ID is being
  seen, after setting its truth-value to (stv 1 CONFIDENCE) and increasing
  its sti.
"
  (record-perception (see-face face-id) confidence)
)

(define (perceive-emotion face-id emotion-type confidence)
"
  perceive-emotion FACE-ID EMOTION-TYPE CONFIDENCE

  Returns the atom representing whether the face with id FACE-ID is in an
  emotional-state EMOTION-TYPE, after increasing its sti and setting its
  truth-value to (stv 1 CONFIDENCE).
"
  (record-perception (face-emotion face-id emotion-type) confidence)
)


(define (perceive-face-talking face-id confidence)
"
  perceive-face-talking FACE-ID CONFIDENCE

  Returns the atom representing that the face with id FACE-ID is talking,
  after increasing its sti, setting its truth-value to (stv 1 NEW-CONF),
  and recording the start/stop time of the face starting/stoping talking.

  When NEW-CONF increases past 0.5 then the time is recorded as
  the start time for the event of the person starting talking, and when
  NEW-CONF decreases past 0.5 the time is recorded as the stop time for
  event of the person stopped talking.

  If FACE-ID = \"\" then an unidentified source is talking.
"
  (record-perception (face-talking face-id) confidence)
)

(define (perceive-eye-state face-id eye-id confidence)
"
  perceive-eye-state FACE-ID EYE-ID

  Return the atom used to represent whether FACE-ID face's EYE-ID eye is open,
  after updating its stv and recording perception time and giving it an
  ecan stimulation.
"
  (record-perception (eye-open face-id eye-id) confidence)
)

(define (perceive-neck-dir dir)
"
  perceive-neck-dir DIR

  Return the atom used to represent which direction of the neck has turned.
"
  (record-perception (looking dir) 1)
)

; --------------------------------------------------------------
(define hook-perceive-word (make-hook 0))
(define (perceive-word-hook)
"
  perceive-word-hook

  Returns the hook that is run when 'perceive-word' is called.
"
  hook-perceive-word
)

(define (perceive-word face-id word)
"
  perceive-word FACE-ID WORD

  If FACE-ID = \"\" then an unidentified source is talking.

  Returns (WordNode WORD) after increasing its sti.
"
  ;TODO: How to represent word said by face-id without having an
  ; explosion of atoms.
  (define wn (Word word))
  (define cn (Concept word))
  (if percep (begin
    (set-time-perceived! wn (FloatValue (current-time-us)))
    (run-hook hook-perceive-word)
    (perception-stimulate wn)
    (perception-stimulate cn)))
  wn
)

; --------------------------------------------------------------
; Utilities for the predicates & schemas
; --------------------------------------------------------------
; NOTE: An event is defined to have occured when the model used to
; represent it becomes true, and when the model stops being true then the
; state that was transitioned into have stopped.

; The confidence value that defines when an event occurs or stops occurring.
(define event-threshold 0.5)
; Key used to record the time an event occured at.
(define event-start (Predicate "event-start-time"))
; Key used to record the time an event stopped occuring at.
(define event-stop (Predicate "event-stop-time"))
; The refractory period in seconds. This is the period in which certain
; predicates will not be returning true after an event transition was made.
(define refractory-period 1)
; This is the window within which a transition is occuring.
(define event-period (/ refractory-period 2))
(define (set-activation-period!)
  (set! event-period (/ refractory-period 2))
)

(define (percep-refractory-period)
"
  percep-refractory-period

  Returns the refractory-period used by the perception module.
"
  refractory-period
)

(define (set-percep-refractory-period! secs)
"
  set-percep-refractory-period! SECS

  Sets the refractory-period of the perception module to SECS.
"
  (set! refractory-period secs)
  (set-activation-period!)
)

(define (true-transition-occurs? old-value new-value)
"
  true-transition-occurs? OLD-VALUE NEW-VALUE

  Returns #t if NEW-VALUE will cause the underlying model to become true,
  else it returns #f. #f is returned even if OLD-VALUE is already in a
  true state because an event occurs only once.
"
  (if (true-value? old-value)
    #f
    (true-value? new-value)
  )
)

(define (false-transition-occurs? old-value new-value)
"
  false-transition-occurs? OLD-VALUE NEW-VALUE

  Returns #t if NEW-VALUE will cause the underlying model to become false,
  else it returns #f. #f is returned even if OLD-VALUE is already in a
  false state because an event occurs only once.
"
  (if (true-value? old-value)
    (not (true-value? new-value))
    #f
  )
)

; TODO Move the time related helpers to the time-server. Some of this
; utilities should have been provided by it.
(define (set-event-times! model old-value new-value time)
"
  set-event-times! MODEL OLD-VALUE NEW-VALUE TIME

  Record the time TIME that MODEL transitions to true or false.
"
  (cond
    ((true-transition-occurs? old-value new-value)
      (cog-set-value! model event-start time))
    ((false-transition-occurs? old-value new-value)
      (cog-set-value! model event-stop time))
    (else model)
  )
)

(define (time-true-transition model)
"
  time-true-transition MODEL

  Returns the time in seconds of the last time the MODEL transitioned
  into a true state. If no time had been recorded then nil is returned.
"
  (let ((time (cog-value model event-start)))
    (if (nil? time)
      time
      (cog-value-ref time 0)
    )
  )
)

(define (time-false-transition model)
"
  time-false-transition MODEL

  Returns the time in seconds of the last time the MODEL transitioned
  into a false state. If no time had been recorded then nil is returned.
"
  (let ((time (cog-value model event-stop)))
    (if (nil? time)
      time
      (cog-value-ref time 0)
    )
  )
)

(define (is-recent-transition-true? model)
"
  is-recent-transition-true? MODEL

  Return #t if the recent state transition of MODEL is a true-transition.
"
  (let ((true-t (time-true-transition model))
    (false-t (time-false-transition model)))
    (cond
      ((nil? true-t) #f)
      ((nil? false-t) #t)
      (else (> true-t false-t)))
  )
)

(define (is-recent-transition-false? model)
"
  is-recent-transition-false? MODEL

  Return #t if the recent state transition of MODEL is a false-transition.
"
  (let ((true-t (time-true-transition model))
    (false-t (time-false-transition model)))
    (cond
      ((nil? false-t) #f)
      ((nil? true-t) #t)
      (else (< true-t false-t)))
  )
)

(define (since-true-transition-occurred? model secs)
"
  since-true-transition-occurred? MODEL SECS

  Return #t if the time passed since the MODEL had a true transition is
  greater than SECS, and there hasn't been a false transition. If not,
  returns #f.
"
  (if (is-recent-transition-true? model)
    (> (- (current-time-us) (time-true-transition model)) secs)
    #f
  )
)

(define (since-false-transition-occurred? model secs)
"
  since-false-transition-occurred? MODEL SECS

  Return #t if the time passed since the MODEL had a false transition is
  greater than SECS, and there hasn't been a true transition. If not,
  returns #f.
"
  (if (is-recent-transition-false? model)
    (> (- (current-time-us) (time-false-transition model)) secs)
    #f
  )
)

(define (true-transition-occurring? model secs)
"
  true-transition-occurring? MODEL SECS

  Return #t if the time passed since the MODEL had a true transition is
  less than SECS, and there hasn't been a false transition. If not,
  returns #f.
"
  (if (is-recent-transition-true? model)
    (< (- (current-time-us) (time-true-transition model)) secs)
    #f
  )
)

(define (false-transition-occurring? model secs)
"
  false-transition-occurring? MODEL SECS

  Return #t if the time passed since the MODEL had a false transition is
  less than SECS, and there hasn't been a true transition. If not,
  returns #f.
"
  (if (is-recent-transition-false? model)
    (< (- (current-time-us) (time-false-transition model)) secs)
    #f
  )
)

; --------------------------------------------------------------
(define (true-value? value)
"
  true-value? VALUE

  This is an abstraction to make it easier to keep consistent definition
  of what a true value is, and is used internally. Here a value is a
  measure of trueness as per the model defined in here.
"
  (cond
    ((> event-threshold value) #f)
    ((<= event-threshold value) #t))
)

(define (is-model-true? model)
; It is better to find the product of the strength and confidence but for now
; confidence is used as the default stv for new atoms is (stv 1 0), so
; need to waste cpu cycles.
  (let ((conf (cog-confidence model)))
    (if (true-value? conf)
      (stv 1 1)
      (stv 0 1)
    )
  )
)

(define (any-model-true? model-list)
  (let ((true-models (filter is-model-true? model-list)))
    (if (nil? true-models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

(define (negate-stv! tv)
"
  negate-stv! TV

  Returns (stv 1 1) if TV is (stv 0 1) and vice-versa.
"
  (cond
    ((equal? (stv 1 1) tv) (stv 0 1))
    ((equal? (stv 0 1) tv) (stv 1 1))
    (else (error "negate-stv! expected (stv 1 1)/(stv 0 1) got=~a\n" stv))
  )
)

; --------------------------------------------------------------
; These allow adding time based predicates.
(define time-key (Predicate "time-perceived"))

; Default time interval used as a window, backward from current time,
; for which the perception is considered valid. This is in seconds.
(define dti 2)
(define dti-node (Number dti))

(define (set-dti! sec)
"
  set-dti! SEC

  Set the default-time-interval(dti) used as a time window for considering
  a valid perception and returns the NumberNode that represents it.
"
  (if (not (number? sec))
    (error "Only numbers should be passed as argument"))

  (set! dti sec)
  (set! dti-node (Number sec))
)

(define (get-dti)
"
  get-dti

  Returns the NumberNode used to represent the default-time-interval(dti)
  used as a time window for considering a valid perception.
"
 dti-node
)

(define (current-time-us)
"
  Returns the current-time including microseconds by converting the pair
  returned by (gettimeofday) into a floating number representing seconds
  with microsecond accuracy.
"
  (let ((time (gettimeofday)))
    (+ (car time) (/ (cdr time) 1000000))
  )
)

(define (set-time-perceived! atom time)
"
  set-time-perceived! ATOM TIME

  Record TIME as the time of perception of ATOM, and return ATOM.
"
  (cog-set-value! atom time-key time)
)

(define (time-perceived atom)
"
  time-perceived ATOM

  Return the time of perception of the ATOM, or nil. Nil is used to denote
  that there is no perception time associated with ATOM.
"
  ; If a perception time was not set then it is assumed to have not been
  ; perceived and nil is returned.
  (let ((time (cog-value atom time-key)))
    (if (nil? time)
      time
      (cog-value-ref time 0)
    )
  )
)

(define (time-within-interval? time end-time time-interval)
"
  time-within-interval? TIME END-TIME TIME-INTERVAL

  Returns #t if (END-TIME - TIME-INTERVAL) <= TIME <= END-TIME, else it
  returns #f. All times passed as argument should be in seconds.
"
  (and
    (<= (- end-time time-interval) time)
    (<= time end-time))
)

(define (perceived? atom end-time time-interval)
"
  perceived? ATOM END-TIME TIME-INTERVAL

  Returns (stv 1 1) if
    (END-TIME - TIME-INTERVAL) <= time-perceived <= END-TIME, else it
  returns (stv 0 1). All times passed as argument should be in seconds.
"
  (let ((time (time-perceived atom)))
    (if (and (not (nil? time))
          (time-within-interval? time end-time time-interval))
      (stv 1 1)
      (stv 0 1)
    )
  )
)

(define* (was-perceived? atom #:optional (time-interval dti-node))
"
  was-perceived? ATOM TIME-INTERVAL

  Returns (stv 1 1) if
    (current-time - TIME-INTERVAL) <= time-perceived <= current-time, else it
  returns (stv 0 1). All times passed as argument should be in seconds.
"
  (perceived? atom (current-time-us)
    (string->number (cog-name time-interval)))
)

; --------------------------------------------------------------
(define timer-predicate (Predicate "timer-predicate"))
(define timer-last-stimulated 0)

(define (ghost-stimulate-timer)
"
  Stimulate the timer predicate, so that the rules having time-related
  predicates will likely have some non-zero STI.

  Currently the stimulus will be proportional to the elapsed time (sec)
  since last time it's called.
"
  (if (> timer-last-stimulated 0)
    (cog-stimulate timer-predicate
      (* 10 (- (current-time) timer-last-stimulated))))

  (set! timer-last-stimulated (current-time))
)

; --------------------------------------------------------------
(define perception-stimulus 150)
(define (perception-stimulate . atoms)
"
  perception-stimulate ATOMS

  Stimulate the given list of ATOMS with perception's  default stimulus
  amount.
"
  (map (lambda (a) (cog-stimulate a perception-stimulus)) atoms)
)

; --------------------------------------------------------------
(define (get-rule-from-alias alias)
"
  get-rule-from-alias ALIAS

  Get the psi-rule with ALIAS, assuming ALIAS is unique
  among all the psi-rules.
"
  (car (filter psi-rule?
    (cog-chase-link 'ListLink 'ImplicationLink
      (Concept alias)))))

; --------------------------------------------------------------
; Define the logger for actions.
(define schema-logger (cog-new-logger))

; Default configuration for the perception logger
(cog-logger-set-component! schema-logger "ACTION")
(cog-logger-set-level! schema-logger "info")
(cog-logger-set-stdout! schema-logger #t)

(define (action-logger)
"
  Return the logger for actions.
"
  schema-logger
)

(define (print-by-action-logger action-node . str-nodes)
"
  say ACTION-NODE . STR-NODES

  Display the string that forms the name of the nodes ACTION-NODE and
  STR-NODE to stdout with the format of \"ACTION-NODE-name: STR-NODE-name\".
"
  (cog-logger-set-component! schema-logger (cog-name action-node))
  (cog-logger-info schema-logger "~a" (map cog-name str-nodes))
  fini
)

; --------------------------------------------------------------
; The Node that represents sources
(define source-node (Concept "source"))

; Define keys used by sources.
;; Key for query passed to a source.
(define source-query-key (Predicate "query"))
;; Key used for recording the function name that is called with the query.
(define source-func-name-key (Predicate "func-name"))
;; Key used for recording the latest sentence passed for processing.
(define source-latest-sent-key (Predicate "last-sentence"))

; Will be used by the 'get_answer' and 'get_answer_source' schemas
; to indicate where does the answer come from
(define answer-src '())

(define* (def-source name func-name #:optional (input-type 'string)
                     (output-type 'string))
"
  def-source NAME FUNC-NAME [INPUT-TYPE] [OUTPUT-TYPE]

  Returns the atom that represents a source that is identified by NAME after
  recording the name of the function, FUNC-NAME, that is an interface to
  the source. INPUT-TYPE and OUTPUT-TYPE are set to 'string by default.

  TODO: Define other types of inputs and outputs.
"
  (define src (Concept name))
  (Inheritance src source-node)
  (cog-set-value! src source-func-name-key (StringValue func-name))
)

(define (source-func-name source)
"
  source-func-name SOURCE

  Returns the name of the function that acts the interface for SOURCE.
"
  (cog-value-ref (cog-value source source-func-name-key) 0)
)

(define (source-set-query! sent query)
"
  source-set-query! SENT QUERY

  Returns SENT after recording the QUERY, which is a string.
"
  (cog-set-value! sent source-query-key (StringValue query))
)

(define (source-query sent)
"
  source-query SENT

  Returns the query string that is extracted from SENT and is to be processed,
  or is being processed, or was processed. SENT will identify a single query.
"
  (cog-value-ref (cog-value sent source-query-key) 0)
)

(define (source-set-processing! source sent tv)
"
  source-set-processing? SOURCE SENT TV

  Returns SOURCE after setting its processing state, for the query extracted
  from SENT, to the simple truth value TV. The value passed should be
  either (stv 1 1) or (stv 0 1)
"
  (cog-set-value! source sent tv)
)

(define (source-processing? source sent)
"
  source-processing? SOURCE SENT

  Returns (stv 1 1) if SOURCE is processing a query extracted from SENT
  and (stv 0 1) if not.
"
  (cog-value source sent)
)

(define (source-set-result! sent source result)
"
  source-set-result SENT SOURCE RESULT

  Record the RESULT from SOURCE for the query extracted from SENT.
  An empty string is used to represent no result.

  It also signals that processing is completed.
"
  ; The order here matters for source-has-result?
  (set-value! sent source (StringValue result))
  (source-set-processing! source sent (stv 0 1))
  *unspecified*
)

(define (source-result sent source)
"
  source-result SENT SOURCE

  Returns the result value for the sentence SENT processed by SOURCE.
"
  (get-value sent source 0)
)

(define (source-has-result? sent source)
"
  source-has-result? SENT SOURCE

  Returns (stv 1 1) if SOURCE has a result for the query extracted from SENT
  else it returns (stv 0 1).
"
  (let ((sp (source-processing? source sent)))
    ; The order here is dependent on source-set-result!
    (cond
      ((or (equal? '() sp) (equal? (stv 1 1) sp)) (stv 0 1))
      (else (stv 1 1))
    )
  )
)

(define (set-value! sent source value)
"
  set-value! SENT SOURCE VALUE

  Returns SENT after associating SOURCE with VALUE. It differs from cog-value
  b/c it uses a LinkValue of values so as to store time information.
"
  (cog-set-value! sent source (LinkValue value (FloatValue (current-time-us))))
)

(define (get-value sent source index)
"
  get-value SENT SOURCE INDEX

  Returns the value at INDEX of the LinkValue, that is returned when
  running (cog-value SENT SOURCE).
"
  (let ((link-value (cog-value sent source)))
    (if (nil? link-value)
      link-value
      (cog-value-ref link-value index))
  )
)

(define (get-sources)
"
  get-sources

  Returns a list of atoms that represent sources, or nil if their are no
  sources defined.
"
  (cog-outgoing-set (cog-execute!
    (Get
      (TypedVariable
        (Variable "source")
        (Type "ConceptNode"))
      (Inheritance (Variable "source") source-node))
  ))
)

(define-public (load-trail-3)
  (load-from-path
    (string-append "opencog/ghost/procedures/" "pln-reasoner.scm"))
  (load-from-path
    (string-append "opencog/ghost/procedures/" "pln-trail-3.scm"))
  (load-from-path
    (string-append "opencog/ghost/procedures/" "pln-utils.scm"))
)
; --------------------------------------------------------------
; Because macros require all the bindings used before expansion load
; the files last.
(load "procedures/focus-set.scm")
(load "procedures/predicates.scm")
(load "procedures/schemas.scm")
(load "procedures/sq-bind.scm")
; TODO: move genric steps to the pln module
;(load "procedures/pln-reasoner.scm")
;(load "procedures/pln-trail-3.scm")
;(load "procedures/pln-utils.scm")


