;; This is the GHOST action selector for finding and deciding
;; which action should be executed at a particular point in time.

(define* (eval-and-select RULES #:optional (SKIP-STI #f))
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

  SKIP-STI is for backward compatibility, used to decide whether to
  include STI of a rule (Icag) in action selection or not. It's needed
  as the default STI is zero. So if one runs GHOST without running
  ECAN (as the earlier version of GHOST permits) then no action will
  ever be triggered.
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
    (define sti (if SKIP-STI
      ; Weight higher if the rule is in the current topic, and/or
      ; it's a rejoinder
      (let ((topic-weight
              (if (is-rule-in-topic? R (ghost-get-curr-topic)) 1 0.5))
            (rejoinder-weight
             ; A crude way to see if R is a rejoinder
             (if (any (lambda (x) (equal? x ghost-last-executed))
                      (cog-get-all-nodes R))
                 1 0.1)))
        (* topic-weight rejoinder-weight))
      (if (> sti-weight 0)
        (* sti-weight (cog-av-sti R)) 1)))
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

  ; ----------
  (for-each
    (lambda (r)
      ; Skip the rule if its STI or strength is zero
      (if (or SKIP-STI
              (and (> (cog-av-sti r) 0) (> (cog-stv-strength r) 0)))
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
                "Skipping action with zero weight: ~a" ra))))
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
           (map gar (cog-incoming-by-type ghost-no-constant 'MemberLink)))))
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
         (selected (eval-and-select (delete-duplicates
           (append exact-match no-const dual-match)) #t)))

        (cog-logger-debug ghost-logger "For input:\n~a" input-lseq)
        (cog-logger-debug ghost-logger "Rules with no constant:\n~a" no-const)
        (cog-logger-debug ghost-logger "Exact match:\n~a" exact-match)
        (cog-logger-debug ghost-logger "Dual match:\n~a" dual-match)
        (cog-logger-debug ghost-logger "To-be-evaluated:\n~a" rules-candidates)
        (cog-logger-debug ghost-logger "Selected:\n~a" selected)

        (List selected)))

; ----------
(define-public (ghost-get-rules-from-af)
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

  (define candidate-rules
    (if ghost-af-only?
      (filter is-psi-rule? (cog-af))
      (psi-get-rules ghost-component)))
  (define rule-selected (eval-and-select candidate-rules))

  ; Stimulate the timer predicate
  (ghost-stimulate-timer)

  (cog-logger-debug ghost-logger "Candidate Rules:\n~a" candidate-rules)
  (cog-logger-debug ghost-logger "Selected:\n~a" rule-selected)

  ; Keep a record of which rule got executed, just for rejoinders
  (if (not (null? rule-selected))
    (let ((next-responder (cog-value rule-selected ghost-next-responder))
          (next-rejoinder (cog-value rule-selected ghost-next-rejoinder))
          (av-alist (cog-av->alist (cog-av rule-selected))))
      ; Stimulate the next rules in the sequence and lower the STI of
      ; the current one
      ; Rejoinders will have a bigger boost than responders by default
      (if (not (null? next-responder))
        (for-each
          (lambda (r) (cog-stimulate r (* default-stimulus responder-sti-boost)))
          (cog-value->list next-responder)))
      (if (not (null? next-rejoinder))
        (for-each
          (lambda (r) (cog-stimulate r (* default-stimulus rejoinder-sti-boost)))
          (cog-value->list next-rejoinder)))
      ; Lower the STI of the selected one
      (cog-set-av!
        rule-selected
        (cog-new-av 0
          (cdr (assoc 'lti av-alist)) (cdr (assoc 'vlti av-alist))))))

  (List rule-selected))

; ----------
; The action selector for OpenPsi
(psi-set-action-selector!
  ghost-component
  (ExecutionOutput (GroundedSchema "scm: ghost-get-rules-from-af") (List)))
