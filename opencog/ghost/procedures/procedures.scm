(define-module (opencog ghost procedures)
  #:use-module (ice-9 optargs)
  #:use-module (opencog)
  #:use-module (opencog atom-types)
  #:use-module (opencog attention)
  #:use-module (opencog exec)
  #:use-module (opencog pointmem)
  #:use-module (opencog openpsi)
  #:use-module (opencog spacetime)
  #:use-module (opencog ghost)
  #:export (
    ; Sensory input
    perceive-face
    perceive-emotion
    perceive-word
    perceive-face-talking

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

    ; Utilities
    is-model-true?
    any-model-true?
    set-time-perceived! ; temporarily exported
    time-perceived ; temporarily exported
    was-perceived?
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
    (cog-stimulate model default-stimulus)
    (cog-set-tv! model (stv 1 confidence))
  )
)

(define (perceive-word word)
"
  perceive-word WORD

  Returns WORD after increasing its sti.
"
  (define wn (Word word))
  (set-time-perceived! wn)

  ; 'ghost-word-seq' is shared among the rules with word-related pattern
  ; This is mainly to make sure the rules with only a wildcard in the pattern
  ; will also get some non-zero STI.
  ; TODO: Find some better representation for that
  (cog-stimulate (ghost-word-seq-pred) (/ default-stimulus 2))

  (ghost-stimulate wn)
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

(define (was-perceived? atom end-time time-interval)
"
  was-perceived? ATOM END-TIME TIME-INTERVAL

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
