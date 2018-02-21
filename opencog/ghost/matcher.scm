;; This is the GHOST action selector for finding and deciding
;; which action should be executed at a particular point in time.

; Note: This is not being used at the moment
(define (cs-eval RULES)
"
  A ChatScript-like way for selecting actions.

  Partition RULES into different categories based on their type,
  and evaluate each of them. A satisfied one will be returned.
"
  (define curr-topic (ghost-get-curr-topic))
  (define topic-rejoinders '())
  (define topic-responders '())
  (define topic-random-gambits '())
  (define topic-gambits '())
  (define responders '())
  (define random-gambits '())
  (define gambits '())
  (define rules-evaluated '())

  ; To evaluate a list of rules using "psi-satisfiable?"
  (define (eval-rules R)
    (set! rules-evaluated
      (append-map
        (lambda (r)
          (if (equal? (stv 1 1) (psi-satisfiable? r)) (list r) '()))
        R))
    rules-evaluated)

  ; To pick one of the selected rules either randomly or based on their weights
  (define (pick-rule F)
    ; Either randomly pick one of the rules, or pick based on their TV strength
    (cond ((equal? 'RANDOM F)
           (list-ref rules-evaluated
             (random (length rules-evaluated) (random-state-from-platform))))
          ((equal? 'RANK F)
           (let ((highest-strength 0)
                 (selected-rule '()))
             (for-each (lambda (r)
               (let ((strength (cog-stv-strength r)))
                    (if (> strength highest-strength)
                        (begin (set! highest-strength strength)
                               (set! selected-rule r)))))
               rules-evaluated)
             selected-rule))))

  ; Go through the rules and put them into different categories
  (for-each (lambda (r)
    (if (is-rule-in-topic? r curr-topic)
        (cond ((equal? strval-rejoinder (cog-value r ghost-rule-type))
               (set! topic-rejoinders (append topic-rejoinders (list r))))
              ((equal? strval-responder (cog-value r ghost-rule-type))
               (set! topic-responders (append topic-responders (list r))))
              ((equal? strval-random-gambit (cog-value r ghost-rule-type))
               (set! topic-random-gambits (append topic-random-gambits (list r))))
              ((equal? strval-gambit (cog-value r ghost-rule-type))
               (set! topic-gambits (append topic-gambits (list r)))))
        (cond ; Skip the rule if it's in a topic that should be explicitly triggered
              ; A rule may (rarely) be in multiple topics, so check them all
              ((every (lambda (t) (topic-has-feature? t "noaccess")) (get-rule-topic r)))
              ((equal? strval-responder (cog-value r ghost-rule-type))
               (set! responders (append responders (list r))))
              ((equal? strval-random-gambit (cog-value r ghost-rule-type))
               (set! random-gambits (append random-gambits (list r))))
              ((equal? strval-gambit (cog-value r ghost-rule-type))
               (set! gambits (append gambits (list r)))))))
    RULES)

  (cog-logger-debug ghost-logger "Number of RULES = ~a\n" (length RULES))
  (cog-logger-debug ghost-logger "topic-rejoinders = ~a\n" (length topic-rejoinders))
  (cog-logger-debug ghost-logger "topic-responders = ~a\n" (length topic-responders))
  (cog-logger-debug ghost-logger "topic-random-gambits = ~a\n" (length topic-random-gambits))
  (cog-logger-debug ghost-logger "topic-gambits = ~a\n" (length topic-gambits))
  (cog-logger-debug ghost-logger "responders = ~a\n" (length responders))
  (cog-logger-debug ghost-logger "random-gambits = ~a\n" (length random-gambits))
  (cog-logger-debug ghost-logger "gambits = ~a\n" (length gambits))

  ; And finally, evaluate the rules in this order:
  ; 1) topic-rejoinders
  ; 2) topic-responders
  ; 3) topic-random-gambits
  ; 4) topic-gambits
  ; 5) responders
  ; 6) random-gambits
  ; 7) gambits
  (cond ((not (null? (eval-rules topic-rejoinders))) (pick-rule 'RANK))
        ((not (null? (eval-rules topic-responders))) (pick-rule 'RANK))
        ((not (null? (eval-rules topic-random-gambits))) (pick-rule 'RANDOM))
        ((not (null? (eval-rules topic-gambits))) (pick-rule 'RANK))
        ((not (null? (eval-rules responders))) (pick-rule 'RANK))
        ((not (null? (eval-rules random-gambits))) (pick-rule 'RANDOM))
        ((not (null? (eval-rules gambits))) (pick-rule 'RANK))
        ; If we are here, there is no match
        (else (list))))

(define (eval-and-select RULES)
"
  This is the goal-driven action selection.

  Evaluate the candidates and see which of them, if any, satisfy the
  current context, and then select one of them based on their weights.

  The weight of an action will be calculated in this way:
  Wa = 1/Na * sum(Wcagi)

  Na = number of satisfied rules [i] that have the action [a]
  Wcagi = Scag * Sc * Ig * T

  Scag = Strength of the psi-rule (c ∧ a => g)
  Sc = Satisfiability of the context of the psi-rule
  Ig = Importance of the goal [g]
  T = Whether the rule is in the current topic [1, 0.5]
"
  ; Store the evaluation results for the contexts, so that the same context
  ; won't be evaluated again, in the same psi-step
  (define context-alist '())

  ; Store the action counts [Na]
  (define action-cnt-alist '())

  ; Store the sum of action-weights [sum(Wcagi)]
  (define sum-weight-alist '())

  ; Store the final weight of an action [Wa]
  (define action-weight-alist '())

  ; For random number generation
  (define total-weight 0)

  ; Get to know what the current topic is
  (define curr-topic (ghost-get-curr-topic))

  ; For quickly find out which rule an given
  ; action belongs to
  ; To-be removed once action selector actually
  ; returns an action instead of a rule (?)
  (define action-rule-alist '())

  ; Calculate the weight of the rule R
  (define (calculate-rweight R)
    ; XXX TODO: The default STI is zero, which means
    ; the weight for all the rules will be zero
    ; To workaround this, a non-zero STI will be assigned
    ; to the goal for now until we have ECAN and GHOST
    ; running altogether in the near future
    (define sti (cog-av-sti (psi-get-goal R)))

    ; Now calculate the weight
    (* (cog-stv-strength R)
       (assoc-ref context-alist (psi-get-context R))
       (if (= 0 sti) 1 sti)
       ; TODO: Use a more sophisticated way for the below
       (if (is-rule-in-topic? R curr-topic) 1 0.5)))

  ; Calculate the weight of the action A
  (define (calculate-aweight A)
    (* (/ 1 (assoc-ref action-cnt-alist A))
       (assoc-ref sum-weight-alist A)))

  (for-each
    (lambda (r)
      (define rc (psi-get-context r))
      (define ra (psi-get-action r))

      ; Though an action may be in multiple psi-rule, but
      ; it doesn't really matter here, just record one
      ; of them
      (set! action-rule-alist (assoc-set! action-rule-alist ra r))

      (if (equal? (assoc-ref context-alist rc) #f)
        (set! context-alist (assoc-set! context-alist rc
          (cdadr (cog-tv->alist (psi-satisfiable? r))))))

      (if (equal? (assoc-ref action-cnt-alist ra) #f)
        (set! action-cnt-alist (assoc-set! action-cnt-alist ra 1))
        (set! action-cnt-alist (assoc-set! action-cnt-alist ra
          (+ (assoc-ref action-cnt-alist ra) 1))))

      ; Ignore the action if its weight is zero
      (let ((w (calculate-rweight r)))
        (if (> w 0)
          (if (equal? (assoc-ref sum-weight-alist ra) #f)
            (set! sum-weight-alist (assoc-set! sum-weight-alist ra w))
            (set! sum-weight-alist (assoc-set! sum-weight-alist ra
              (+ (assoc-ref sum-weight-alist ra) w))))
          (cog-logger-debug ghost-logger
            "Skipping action with zero weight: ~a" ra))))
    RULES)

  ; Note: sum-weight-alist and action-weight-alist do not contain
  ; actions that have a zero weight
  (for-each
    (lambda (a)
      (set! action-weight-alist
        (assoc-set! action-weight-alist (car a) (calculate-aweight (car a))))
      (set! total-weight (+ total-weight
        (assoc-ref action-weight-alist (car a)))))
    sum-weight-alist)

  (cog-logger-debug ghost-logger "context-alist: ~a" context-alist)
  (cog-logger-debug ghost-logger "action-cnt-alist: ~a" action-cnt-alist)
  (cog-logger-debug ghost-logger "sum-weight-alist: ~a" sum-weight-alist)
  (cog-logger-debug ghost-logger "action-weight-alist: ~a" action-weight-alist)
  (cog-logger-debug ghost-logger "action-rule-alist: ~a" action-rule-alist)

  ; If there is only one action in the list, return that
  ; Otherwise, pick one based on their weights
  ; TODO: Return the actual action instead of a rule
  (if (equal? (length action-weight-alist) 1)
    (assoc-ref action-rule-alist (caar action-weight-alist))
    (let* ((accum-weight 0)
           (cutoff (* total-weight (random:uniform (random-state-from-platform))))
           (action-rtn
             (find
               (lambda (a)
                 (set! accum-weight (+ accum-weight (cdr a)))
                 (<= cutoff accum-weight))
               action-weight-alist)))
      (if (equal? #f action-rtn)
        (list)
        (assoc-ref action-rule-alist (car action-rtn))))))

; ----------
(define-public (ghost-find-rules SENT)
"
  The action selector. It first searches for the rules using DualLink,
  and then does the filtering by evaluating the context of the rules.
  Eventually returns a list of weighted rules that can satisfy the demand.
"
  (let* ((input-lseq (gddr (car (filter (lambda (e)
           (equal? ghost-lemma-seq (gar e)))
             (cog-get-pred SENT 'PredicateNode)))))
         ; The ones that contains no variables/globs
         (exact-match (filter psi-rule? (cog-get-trunk input-lseq)))
         ; The ones that contains no constant terms
         (no-const (filter psi-rule? (append-map cog-get-trunk
           (cog-chase-link 'MemberLink 'ListLink ghost-no-constant))))
         ; The ones found by the recognizer
         (dual-match (filter psi-rule? (append-map cog-get-trunk
           (cog-outgoing-set (cog-execute! (Dual input-lseq))))))
         ; Get the psi-rules associate with them with duplicates removed
         (rules-candidates
           (fold (lambda (rule prev)
                   ; Since a psi-rule can satisfy multiple goals and an
                   ; ImplicationLink will be generated for each of them,
                   ; we are comparing the implicant of the rules instead
                   ; of the rules themselves, and create a list of rules
                   ; with unique implicants
                   (if (any (lambda (r) (equal? (gar r) (gar rule))) prev)
                       prev (append prev (list rule))))
                 (list) (append exact-match no-const dual-match)))
         ; Evaluate the matched rules one by one and see which of them satisfy
         ; the current context
         ; One of them, if any, will be selected and executed
         (selected (eval-and-select
           (delete-duplicates (append exact-match no-const dual-match)))))

        (cog-logger-debug ghost-logger "For input:\n~a" input-lseq)
        (cog-logger-debug ghost-logger "Rules with no constant:\n~a" no-const)
        (cog-logger-debug ghost-logger "Exact match:\n~a" exact-match)
        (cog-logger-debug ghost-logger "Dual match:\n~a" dual-match)
        (cog-logger-debug ghost-logger "To-be-evaluated:\n~a" rules-candidates)
        (cog-logger-debug ghost-logger "Selected:\n~a" selected)

        ; Keep a record of which rule got executed, just for rejoinders
        ; TODO: Move this part to OpenPsi?
        ; TODO: This should be created after actually executing the action
        (if (not (null? selected))
          (State ghost-last-executed (psi-rule-alias selected)))

        (List selected)))

(Define
  (DefinedSchema (ghost-prefix "Get Current Input"))
  (Get (State ghost-curr-proc (Variable "$x"))))

(Define
  (DefinedSchema (ghost-prefix "Find Rules"))
  (Lambda (VariableList (TypedVariable (Variable "$sentence")
                                       (Type "SentenceNode")))
          (ExecutionOutput (GroundedSchema "scm: ghost-find-rules")
                           (List (Variable "$sentence")))))

; The action selector for OpenPsi
(psi-set-action-selector!
  ghost-component
  (Put (DefinedSchema (ghost-prefix "Find Rules"))
       (DefinedSchema (ghost-prefix "Get Current Input"))))
