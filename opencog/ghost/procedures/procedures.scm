(define-module (opencog ghost procedures)
  #:use-module (ice-9 optargs)
  #:use-module (opencog)
  #:use-module (opencog atom-types)
  #:use-module (opencog attention)
  #:use-module (opencog logger)
  #:use-module (opencog exec)
  #:use-module (opencog openpsi)
  #:export (
    ; Sensory input
    perceive-face
    perceive-emotion
    perceive-word
    perceive-face-talking
    perceive-eye-state

    ; Sensory input hooks
    perceive-word-hook

    ; Perceptual predicates
    person_appears
    person_smiles
    person_angry
    person_talking
    person_not_talking
    word_perceived

    ; Time related predicates
    after_min
    after_user_started_talking
    after_user_stopped_talking

    ; schemas
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

    ; Utilities
    set-dti!
    get-dti
    is-model-true?
    any-model-true?
    set-time-perceived! ; temporarily exported
    time-perceived ; temporarily exported
    was-perceived?
    ghost-stimulate-timer
    action-logger
  )
)

(load "procedures/predicates.scm")
(load "procedures/schemas.scm")

; --------------------------------------------------------------
; There is a spacemap in eva-model module called faces. Thus the rename.
; (define facemap (SpaceMapNode "perceived-faces"))

; --------------------------------------------------------------
; APIs used to create the atoms used to represent the world, aka
; world-model. These are not exported.
; --------------------------------------------------------------
(define (see-face face-id)
"
  Define the atom used to represent that the face represented by FACE-ID
  is being seen.
"
  (Evaluation
    (Predicate "see")
    (List
      (Concept "I")
      (Concept face-id)))
)

(define (face-emotion face-id emotion-type)
"
  Define the atom used to represent EMOTION-TYPE of the face with
  id FACE-ID.
"
  (Evaluation
    (Predicate emotion-type)
    (List
      (Concept face-id)))
)

(define (face-talking face-id)
"
  Define the atom used to represent that the face represented by FACE-ID
  is talking.
"
  (Evaluation
    (Predicate "talking")
    (List
      (Concept face-id)))
)

(define face-talking-sign
  (Signature
    (Evaluation
      (Predicate "talking")
      (List
        (Type "ConceptNode"))))
)

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
      (Variable "face-talking"))
  ))
)

; --------------------------------------------------------------
; APIs for inputing sensory information.
; --------------------------------------------------------------
;(define (perceived-face face-id x y z)
;  (cog-pointmem-map-atom facemap (Concept face-id)
;    (List (Number x) (Number y) (Number z)))
(define default-stimulus 150)

(define (perceive-face face-id confidence)
"
  perceive-face FACE-ID CONFIDENCE

  Returns the atom representing whether the face with id FACE-ID is being
  seen, after setting its truth-value to (stv 1 CONFIDENCE) and increasing
  its sti.
"
  (let ((model (see-face face-id)))
    (set-time-perceived! model)
    (cog-stimulate model default-stimulus)
    (cog-set-tv! model (stv 1 confidence))
  )
)

(define (perceive-emotion face-id emotion-type confidence)
"
  perceive-emotion FACE-ID EMOTION-TYPE CONFIDENCE

  Returns the atom representing whether the face with id FACE-ID is in an
  emotional-state EMOTION-TYPE, after increasing its sti and setting its
  truth-value to (stv 1 CONFIDENCE).
"
  (let ((model (face-emotion face-id emotion-type)))
    (set-time-perceived! model)
    (cog-stimulate model default-stimulus)
    (cog-set-tv! model (stv 1 confidence))
  )
)

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

  Returns WORD after increasing its sti.
"
  ;TODO: How to represent word said by face-id without having an
  ; explosion of atoms.
  (define wn (Word word))
  (define cn (Concept word))
  (set-time-perceived! wn)
  (run-hook hook-perceive-word)
  (perception-stimulate wn)
  (perception-stimulate cn)
)

(define (perceive-face-talking face-id new-conf)
"
  perceive-face-talking FACE-ID NEW-CONF

  Returns the atom representing that the face with id FACE-ID is talking,
  after increasing its sti, setting its truth-value to (stv 1 NEW-CONF),
  and recording the start/stop time of the face starting/stoping talking.

  When NEW-CONF increases past 0.5 then the time is recorded as
  the start time for the event of the person starting talking, and when
  NEW-CONF decreases past 0.5 the time is recorded as the stop time for
  event of the person stopped talking.

  If FACE-ID = \"\" then an unidentified source is talking.
"
  (let* ((model
           (if (equal? "" face-id) face-talking-sign (face-talking face-id)))
    (old-conf (tv-conf (cog-tv model))))

    (cog-set-tv! model (stv 1 new-conf))
    (cog-stimulate model default-stimulus)
    (set-event-times! face-talking-sign model old-conf new-conf)
  )
)

(define (perceive-eye-state face-id eye-id confidence)
"
  perceive-eye-state FACE-ID EYE-ID

  Return the atom used to represent whether FACE-ID face's EYE-ID eye is open,
  after updating its stv and recording perception time and giving it an
  ecan stimulation.
"
  (let ((model (eye-open face-id eye-id)))
    (set-time-perceived! model)
    (cog-stimulate model default-stimulus)
    (cog-set-tv! model (stv 1 confidence))
  )
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
(define (set-event-times! sign model old-value new-value)
"
  set-event-times! SIGN MODEL OLD-VALUE NEW-VALUE

  Record the time that MODEL transitions to true or false. Also, records
  the time of the recent transition for the set of models having the same
  pattern as MODEL and represented in SIGN, a SignatureLink representing
  the patterns.
"
  (cond
    ((true-transition-occurs? old-value new-value)
      (cog-set-value! sign event-start (FloatValue (current-time-us)))
      (cog-set-value! model event-start (FloatValue (current-time-us))))
    ((false-transition-occurs? old-value new-value)
      (cog-set-value! sign event-stop (FloatValue (current-time-us)))
      (cog-set-value! model event-stop (FloatValue (current-time-us))))
    (else model)
  )
)

(define (time-true-transition model)
"
  time-true-transition MODEL

  Returns the time in seconds of the last time the MODEL transitioned
  into a true state. If no time had been recorded then nil is returned.
  MODEL could be a SignatureLink representing the patterns of the models.
  If a SignatureLink is used then it would give the time for the recent
  true transition for the set of models which have the same pattern as
  MODEL.
"
  (let ((time (cog-value model event-start)))
    (if (null? time)
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
  MODEL could be a SignatureLink representing the patterns of the models.
  If a SignatureLink is used then it would give the time for the recent
  false transition for the set of models which have the same pattern as
  MODEL.
"
  (let ((time (cog-value model event-stop)))
    (if (null? time)
      time
      (cog-value-ref time 0)
    )
  )
)

(define (is-recent-transition-true? model)
"
  is-recent-transition-true? MODEL

  Return #t if the recent state transition of MODEL is a true-transition.
  MODEL could be a SignatureLink representing the patterns of the models.
  If a SignatureLink is used then it would return #t if the recent transition
  for any member of the set of models which have the same pattern as MODEL
  had a true-transition.
"
  (let ((true-t (time-true-transition model))
    (false-t (time-false-transition model)))
    (cond
      ((null? true-t) #f)
      ((null? false-t) #t)
      (else (> true-t false-t)))
  )
)

(define (is-recent-transition-false? model)
"
  is-recent-transition-false? MODEL

  Return #t if the recent state transition of MODEL is a false-transition.
  MODEL could be a SignatureLink representing the patterns of the models.
  If a SignatureLink is used then it would return #t if the recent transition
  for any member of the set of models which have the same pattern as MODEL
  had a false-transition.
"
  (let ((true-t (time-true-transition model))
    (false-t (time-false-transition model)))
    (cond
      ((null? false-t) #f)
      ((null? true-t) #t)
      (else (< true-t false-t)))
  )
)

(define (since-true-transition-occurred? model secs)
"
  since-true-transition-occurred? MODEL SECS

  Return #t if the time passed since the MODEL had a true transition is
  greater than SECS, and there hasn't been a false transition. If not,
  returns #f. MODEL could be a SignatureLink representing the patterns of
  the models. If a SignatureLink is used then it would return #t if the check
  is #t for any member of the set of models which have the same pattern
  as MODEL.
"
  (if (is-recent-transition-true? model)
    (>= (- (current-time-us) (time-true-transition model)) secs)
    #f
  )
)

(define (since-false-transition-occurred? model secs)
"
  since-false-transition-occurred? MODEL SECS

  Return #t if the time passed since the MODEL had a false transition is
  greater than SECS, and there hasn't been a true transition. If not,
  returns #f. MODEL could be a SignatureLink representing the patterns of
  the models. If a SignatureLink is used then it would return #t if the check
  is #t  for any member of the set of models which have the same pattern
  as MODEL.
"
  (if (is-recent-transition-false? model)
    (>= (- (current-time-us) (time-false-transition model)) secs)
    #f
  )
)

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
  (let ((conf (tv-conf (cog-tv model))))
    (if (true-value? conf)
      (stv 1 1)
      (stv 0 1)
    )
  )
)

(define (any-model-true? model-list)
  (let ((true-models (filter is-model-true? model-list)))
    (if (null? true-models)
      (stv 0 1)
      (stv 1 1)
    )
  )
)

(define (negate-stv! stv)
"
  negate-stv! STV

  Returns (stv 1 1) if STV is (stv 0 1) and vice-versa.
"
  (cond
    ((equal? (stv 1 1) stv) (stv 0 1))
    ((equal? (stv 0 1) stv) (stv 1 1))
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

(define (set-time-perceived! atom)
"
  set-time-perceived! ATOM

  Record the current time as the time of perception of ATOM, and return ATOM.
"
  (cog-set-value! atom time-key (FloatValue (current-time-us)))
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
    (if (null? time)
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
    (if (and (not (null? time))
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
(define timer-last-stimulated 0)
(define (ghost-stimulate-timer)
"
  Stimulate the timer predicate, so that the rules having time-related
  predicates will likely have some non-zero STI.

  Currently the stimulus will be proportional to the elapsed time (sec)
  since last time it's called.
"
  (if (> timer-last-stimulated 0)
    (cog-stimulate (Concept "timer-predicate")
      (* 10 (- (current-time) timer-last-stimulated))))

  (set! timer-last-stimulated (current-time))
)

; --------------------------------------------------------------
(define perception-stimulus 150)
(define-public (perception-stimulate . atoms)
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
  schema-logger)

(define (print-by-action-logger action-node str-node)
"
  say ACTION-NODE STR-NODE

  Display the string that forms the name of the nodes ACTION-NODE and
  STR-NODE to stdout with the format of \"ACTION-NODE-name: STR-NODE-name\".
"
  (cog-logger-set-component! schema-logger (cog-name action-node))
  (cog-logger-info schema-logger (cog-name str-node))
  fini
)
