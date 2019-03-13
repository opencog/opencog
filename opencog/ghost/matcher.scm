;; This is the GHOST action selector for finding and deciding
;; which action should be executed at a particular point in time.

(define (not-within-refractory? RULE)
"
  Check if a rule is still within its refractory period or not.
"
  (or (null? (cog-value RULE ghost-time-last-executed))
      (> (- (current-time)
            (car (cog-value->list
              (cog-value RULE ghost-time-last-executed))))
         refractory-period)))

(define (stimulate-next-rules RULE)
"
  Stimulate the next rule in the sequence and the rejoinders as well,
  if applicable, and de-stimulate the selected rule.
"
  ; Keep a record of which rule got executed, just for rejoinders
  (let ((next-reactive-rule (cog-value RULE ghost-next-reactive-rule))
        (next-rejoinder (cog-value RULE ghost-next-rejoinder))
        (av-alist (cog-av->alist (cog-av RULE))))
    ; Stimulate the next rules in the sequence and lower the STI of
    ; the current one
    ; Rejoinders will have a bigger boost than reactive rules by default
    (if (not (null? next-reactive-rule))
      (for-each
        (lambda (r) (cog-stimulate r (* default-stimulus reactive-rule-sti-boost)))
        (cog-value->list next-reactive-rule)))
    (if (not (null? next-rejoinder))
      (for-each
        (lambda (r) (cog-stimulate r (* default-stimulus rejoinder-sti-boost)))
        (cog-value->list next-rejoinder)))
    ; Lower the STI of the selected one
    (cog-set-av!
      RULE
      (cog-new-av 0
        (cdr (assoc 'lti av-alist)) (cdr (assoc 'vlti av-alist)))))
)

(define (handle-rule-features RULE)
"
  Handle each of the GHOST rule-features for the selected rule.
"
  (for-each
    (lambda (k)
      (define key-str (cog-name k))
      (define val (cog-value RULE k))
      (cond
        ((string=? "unkeep" key-str)
         ; 'val' here should be a LinkValue of rule labels
         (for-each
           (lambda (r)
             (cog-set-tv! r (cog-new-stv 0 (cog-stv-confidence r))))
           (append-map (lambda (x) (get-rules-from-label (cog-name x)))
             (cog-value->list val))))
        ((string=? "mark-executed" key-str)
         ; 'val' here should be a LinkValue of rule labels
         (for-each
           (lambda (lb)
             (Evaluation ghost-rule-executed (List lb))
             (for-each
               (lambda (r)
                 (cog-set-value! r
                   ghost-time-last-executed
                     (FloatValue (current-time))))
               (get-rules-from-label (cog-name lb))))
            (cog-value->list val)))
        ((string=? "last-executed" key-str)
         (State ghost-last-executed val))))
    (cog-keys RULE))
)

(define (process-ghost-rule RULE)
"
  A wrapper for doing GHOST specific things that are needed for
  the selected rule.
"
  (handle-rule-features RULE)
  (stimulate-next-rules RULE)
)

(define (eval-and-select RULES)
"
  This is the goal-driven action selection.

  Evaluate the candidates and see which of them, if any, satisfy the
  current context, and then select one of them based on their weights.

  The weight of an action will be calculated in this way:
  Wa = 1/Na * sum(Wcagi)

  Na = number of satisfied rules [i] that have the action [a]
  Wcagi = Scag * Sc * Icag * Ug

  Scag = Strength of the psi-rule (c ∧ a => g)
  Sc = Satisfiability of the context of the psi-rule
  Icag = Importance (STI) of the rule
  Ug = Urge of the goal
"
  ; ----------
  ; Store the evaluation results for the contexts, so that the same context
  ; won't be evaluated again, in the same psi-step
  (define context-alist '())

  ; Store the no. of rules that contain this action [Na]
  (define action-cnt-alist '())

  ; Store the sum of action-weights [sum(Wcagi)]
  (define sum-weight-alist '())

  ; Store the final weight of an action [Wa]
  (define action-weight-alist '())

  ; For random number generation
  (define total-weight 0)

  ; For quickly find out which rule an given
  ; action belongs to
  ; TODO: Remove it once action selector actually
  ; returns an action instead of a rule (?)
  (define action-rule-alist '())

  ; For monitoring the status
  (define rules-eval-cnt 0)
  (define rules-sat-cnt 0)

  ; ----------
  ; Calculate the weight of the rule R [Wcagi]
  (define (calculate-rweight R)
    (define strength
      (if (> strength-weight 0)
        (* strength-weight (cog-stv-strength R)) 1))
    (define context
      (if (> context-weight 0)
        (* context-weight
          (assoc-ref context-alist (psi-get-context R))) 1))
    (define sti
      (if (> sti-weight 0)
        (* sti-weight (cog-av-sti R)) 1))
    (define urge
      (if (> urge-weight 0)
        (* urge-weight (psi-urge (psi-get-goal R))) 1))
    (define weight (* strength context sti urge))
    (cog-logger-debug ghost-logger "The weight of the rule ~a is ~a"
      (psi-rule-alias R) weight)
    weight)

  ; Calculate the weight of the action A [Wa]
  (define (calculate-aweight A)
    (* (/ 1 (assoc-ref action-cnt-alist A))
       (assoc-ref sum-weight-alist A)))

  ; Check if a rule should be skipped or not, based on
  ; its current STI, strength, and the last executed time
  (define (accept-rule? r)
    (and (or (= sti-weight 0) (> (cog-av-sti r) 0))
         (or (= strength-weight 0) (> (cog-stv-strength r) 0))
         (not-within-refractory? r)))

  ; ----------
  (for-each
    (lambda (r)
      ; Skip the rule if its STI or strength is zero,
      ; or if it's still with the refractory period,
      ; unless we choose to ignore their weights
      (if (accept-rule? r)
        (let ((rc (psi-get-context r))
              (ra (psi-get-action r)))
          ; Though an action may be in multiple psi-rule, but it doesn't
          ; really matter here, just record one of them
          ; TODO: Remove it once action selector actually returns an
          ; action instead of a rule (?)
          (set! action-rule-alist (assoc-set! action-rule-alist ra r))

          ; Evaluate the context, and save the result in context-alist
          (if (equal? (assoc-ref context-alist rc) #f)
            (set! context-alist
              (assoc-set! context-alist rc
                (cdadr (cog-tv->alist (psi-satisfiable? r))))))

          ; Count the no. of rules that contain this action, and
          ; save it in action-cnt-alist
          (if (equal? (assoc-ref action-cnt-alist ra) #f)
            (set! action-cnt-alist (assoc-set! action-cnt-alist ra 1))
            (set! action-cnt-alist (assoc-set! action-cnt-alist ra
              (+ (assoc-ref action-cnt-alist ra) 1))))

          ; Calculate the weight of this rule
          ; Save and accumulate the weight of an action in sum-weight-alist
          ; Skip the action if its weight is zero, so that sum-weight-alist
          ; and action-weight-alist do not contain actions that have a zero weight
          (let ((w (calculate-rweight r)))
            (set! rules-eval-cnt (1+ rules-eval-cnt))
            (if (> w 0)
              (begin
                (set! rules-sat-cnt (1+ rules-sat-cnt))
                (if (equal? (assoc-ref sum-weight-alist ra) #f)
                  (set! sum-weight-alist (assoc-set! sum-weight-alist ra w))
                  (set! sum-weight-alist (assoc-set! sum-weight-alist ra
                    (+ (assoc-ref sum-weight-alist ra) w)))))
              (cog-logger-debug ghost-logger
                "Skipping action with zero weight: ~a" ra))
          ))
        (cog-logger-debug ghost-logger
          "Skipping rule with zero STI/strength: ~a" r)))
    RULES)

  ; Update the status
  (set! num-rules-found (length RULES))
  (set! num-rules-evaluated rules-eval-cnt)
  (set! num-rules-satisfied rules-sat-cnt)

  ; Finally calculate the weight of an action
  (for-each
    (lambda (a)
      (set! action-weight-alist
        (assoc-set! action-weight-alist (car a) (calculate-aweight (car a))))
      (set! total-weight (+ total-weight
        (assoc-ref action-weight-alist (car a)))))
    sum-weight-alist)

  ; If there is only one action in the list, return that
  ; Otherwise, pick one based on their weights
  ; TODO: Return the actual action instead of a rule
  (if (= (length action-weight-alist) 1)
    (assoc-ref action-rule-alist (caar action-weight-alist))
    ; Here there are special handling for rejoinders:
    ; 1) If there is a rejoinder that satisfy the current context, always trigger it
    ; 2) If there are more than one rejoinders that satisfy the current context,
    ;    always choose the one that is defined first
    (let ((rejoinder
            (fold
              (lambda (rej top-rej)
                (if (and (not (null? (cog-value rej ghost-rej-seq-num)))
                         (or (null? top-rej)
                             (< (car (cog-value->list
                                       (cog-value rej ghost-rej-seq-num)))
                                (car (cog-value->list
                                       (cog-value top-rej ghost-rej-seq-num))))))
                  rej
                  top-rej))
              (list)
              (map (lambda (a) (assoc-ref action-rule-alist (car a))) action-weight-alist))))
      (if (null? rejoinder)
        ; If there is no rejoinder that safisfy the current context, try the reactive rules
        (begin
          (if specificity-based-action-selection
            ; Specificity-based action selection (experimental)
            ; It will select the one with the most specific context, always,
            ; and randomly pick one if there are more than one rules with
            ; the same specificity
            (fold
              (lambda (a rtn)
                (cond
                  ((null? rtn) (assoc-ref action-rule-alist (car a)))
                  ((> (cog-value-ref (cog-value
                        (assoc-ref action-rule-alist (car a)) ghost-context-specificity) 0)
                      (cog-value-ref (cog-value rtn ghost-context-specificity) 0))
                   (assoc-ref action-rule-alist (car a)))
                  (else rtn)))
              (list)
              action-weight-alist
            )
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
                (assoc-ref action-rule-alist (car action-rtn))
              )
            )
          )
        )
        rejoinder))))

; ----------
(define-public (ghost-find-rules)
"
  The action selector that works with ECAN.
  It evaluates and selects psi-rules from the attentional focus.
"
  ; is-psi-rule? uses try-catch so as to avoid getting of
  ; "#<Invalid handle>" error due to change in the value of StateLinks
  ; by a different thread, after this thread got a copy of the atoms in
  ; the attention-focus.
  (define (is-psi-rule? x)
    (catch #t
      (lambda () (psi-rule? x))
      (lambda (key . args)
        (format #f "Catched Error at ~a\nError details =\"~a ~a\"\n"
          (current-source-location) key args) #f)))

  (process-ghost-buffer)

  ; First of all, run the "limbic system" -- a subset of rules that
  ; will always be evaluated, and if appropriate, triggered immediately
  (for-each
    (lambda (p-rule)
      (if (and (not-within-refractory? p-rule)
               (> (cog-tv-mean (psi-satisfiable? p-rule)) 0)
               (> (cog-stv-strength p-rule) 0))
        (psi-imply p-rule)))
    (filter psi-rule? (cog-incoming-set (ConceptNode "Parallel-Rules"))))

  (let* ((candidate-rules
           (if ghost-af-only?
             (filter is-psi-rule? (cog-af))
             (psi-get-rules ghost-component)))
         (rule-selected (eval-and-select candidate-rules)))
    ; Stimulate the timer predicate
    (ghost-stimulate-timer)

    (cog-logger-debug ghost-logger "Candidate Rules:\n~a" candidate-rules)
    (cog-logger-debug ghost-logger "Selected:\n~a" rule-selected)

    ; Do some GHOST specific processing if a satisfiable GHOST rule is found
    (if (not (null? rule-selected)) (process-ghost-rule rule-selected))

    (List rule-selected)))

; ----------
; The action selector for OpenPsi
(psi-set-action-selector!
  ghost-component
  (ExecutionOutput (GroundedSchema "scm: ghost-find-rules") (List)))
