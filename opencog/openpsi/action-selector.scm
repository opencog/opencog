;
; action-selector.scm
;
; The action selector chooses one (or more) rules to run.
;
; Copyright (C) 2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud

; ----------------------------------------------------------------------
(define (psi-set-action-selector! component exec-term)
"
  psi-set-action-selector COMPONENT EXEC-TERM
    Sets EXEC-TERM as the function used to select rules for the COMPONENT.

    EXEC-TERM should be an executable atom.
    COMPONENT should be a node representing a component.
"
  (psi-set-func! exec-term "#f" component "action-selector")
)

; ----------------------------------------------------------------------
(define (psi-action-selector component)
"
  psi-action-selector COMPONENT
    Gets the action-selector of the COMPONENT.
"
  (psi-func component "action-selector")
)

; --------------------------------------------------------------
(define (sort-by-weight RULE-LIST WEIGHT-FN)
"
  sort-by-weight RULE-LIST WEIGHT-FN
    Sort RULE-LIST, according to the weights assigned by the WEIGHT-FN
    function.
"
  (sort! RULE-LIST (lambda (RA RB) (> (WEIGHT-FN RA) (WEIGHT-FN RB))))
)

; --------------------------------------------------------------
(define (rule-sc-weight RULE)
"
  rule-sc-weight RULE
    Return the weight of RULE, which is defined as product of strength
    and confidence.
"
  (let ((rule-stv (cog-tv RULE)))
    (* (tv-conf rule-stv) (tv-mean rule-stv))
  )
)

; --------------------------------------------------------------
(define (rule-sca-weight RULE)
"
  rule-sca-weight RULE
    Return the weight of RULE, which defined as product of strength,
    confidence, and short-term-importance.
"
  (let ((a-stv (cog-tv RULE)))
    (* (tv-conf a-stv) (tv-mean a-stv) (cog-av-sti RULE))
  )
)

; --------------------------------------------------------------
(define (pick-from-weighted-list ALIST WEIGHT-FUNC)
"
  pick-from-weighted-list ALIST WEIGHT-FUNC
    Returns a list containing an item picked from ALIST, based
    on the WEIGHT-FUNC. If ALIST is empty, then returns the empty
    list. If ALIST is not empty, then randomly select from the list,
    with the most heavily weighted, as calcualted by WEIGHT-FUNC.
    The random distribution is a uniform weighted interval
    distribution.
"
  ; Function to sum up the total weights in the list.
  (define (accum-weight rule-list)
    (if (null? rule-list) 0.0
      (+ (WEIGHT-FUNC (car rule-list))
        (accum-weight (cdr rule-list)))))

  ; Recursively move through the list of rules, until the
  ; sum of the weights of the rules exceeds the cutoff.
  ; TODO Why have a cutoff? That is why return a list of rules
  ; instead of just picking the one with the highest weight.
  (define (pick-rule wcut rule-list)
    ; Subtract weight of the first rule.
    (define ncut (- wcut (WEIGHT-FUNC (car rule-list))))

    ; If we are past the cutoff, then we are done.
    ; Else recurse, accumulating the weights.
    (if (<= ncut 0.0)
      (car rule-list)
      (pick-rule ncut (cdr rule-list))))

  (cond
    ; If the list is empty, we can't do anything.
    ((null? ALIST) '())

    ; If there's only one rule in the list, return it.
    ((null? (cdr ALIST)) ALIST)

    ; Else randomly pick among the most-weighted rules.
    (else
      (let* (
        ; Create a list of rules sorted by weight.
        (sorted-rules (sort-by-weight ALIST WEIGHT-FUNC))
        ; The total weight of the rule-list.
        (total-weight (accum-weight sorted-rules))
        ; Pick a number from 0.0 to total-weight.
        (cutoff (* total-weight
          (random:uniform (random-state-from-platform)))))

        (list (pick-rule cutoff sorted-rules))
      ))
  )
)

; --------------------------------------------------------------
(define (psi-select-rules component)
"
  psi-select-rules COMPONENT

  Run the action-selector associated with COMPONENT, and return a list
  of psi-rules. If there aren't any rules returned on execution of the
  action-selector null is returned.
"
  (define (rule-list-flatten rule-list)
    (if (and (equal? (length rule-list) 1)
             (or (equal? 'SeLink (cog-type (car rule-list)))
                 (equal? 'ListLink (cog-type (car rule-list)))))
        (rule-list-flatten (cog-outgoing-set (car rule-list)))
        rule-list
    )
  )

  (let ((acs (psi-action-selector component)))
    (if (null? acs)
      (error
        (format #f "Define an action-selector for component ~a" component))
      (let ((result (cog-execute! acs)))
          (if (null? result)
            '()
            ; Depending on how the action-selector is defined,
            ; the list of psi-rules returned may be nested in
            ; one or more ListLink/SetLink, so remove the
            ; nested links until we reach the actual psi-rules.
            (rule-list-flatten (cog-outgoing-set result))
          )
      )
    )
  )
)
