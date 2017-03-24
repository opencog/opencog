;
; action-selector.scm
;
; The action selector chooses one (or more) rules to run.
;
; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog exec))

(load "demand.scm")
(load "rule.scm")
(load "utilities.scm")

;; --------------------------------------------------------------
;; Dead code, not used anywhere.
;;
;; (define *-act-sel-node-*
;;     (ConceptNode (string-append psi-prefix-str "action-selector")))
;;
;; (define-public (psi-action-selector-set! dsn)
;; "
;;   psi-action-selector-set! EXE
;;
;;   Sets the executable atom to be used for selecting actions.
;;
;;   EXE can be any executable atom. It will be used to select the psi-rules
;;   that will have their actions and goals executed.
;; "
;;     ; Check arguments
;;     (if (and
;;             (not (equal? (cog-type dsn) 'DefinedSchemaNode))
;;             (not (equal? (cog-type dsn) 'ExecutionOutputLink)))
;;         (error "Expected an executable link, got: " dsn))
;;
;;     (StateLink *-act-sel-node-* dsn)
;; )
;;
;; --------------------------------------------------------------
;; (define-public (psi-get-action-selector-generic)
;; "
;;   Returns a list containing the user-defined action-selector.
;; "
;;     (define action-selector-pattern
;;         ; Use a StateLink instead of an InheritanceLink because there
;;         ; should only be one active action-rule-selector at a time,
;;         ; even though there could be multiple possible action-rule
;;         ; selectors.  This enables dynamically changing the
;;         ; action-rule-selector through learning.
;;         (GetLink (StateLink *-act-sel-node-* (Variable "$dpn"))))
;; FIXME -- use cog-chase-link instead of GetLik, here, it is more
;; efficient.
;;
;;     (cog-outgoing-set (cog-execute! action-selector-pattern))
;; )
;;
;; --------------------------------------------------------------
;;
;; (define-public (psi-select-rules)
;; "
;;   psi-select-rules
;;
;;   Return a list of psi-rules that are satisfiable by using the
;;   current action-selector.
;; "
;;     (let ((dsn (psi-get-action-selector-generic)))
;;         (if (null? dsn)
;;             (psi-default-action-selector)
;;             (let ((result (cog-execute! (car dsn))))
;;                 (if (equal? (cog-type result) 'SetLink)
;;                     (cog-outgoing-set result)
;;                     (list result)
;;                 )
;;             )
;;         )
;;     )
;; )
;;
;; --------------------------------------------------------------
;; (define-public (psi-add-action-selector exec-term name)
;; "
;;   psi-add-action-selector EXE NAME
;;
;;   Return the executable atom that is defined ad the atcion selector.
;;
;;   NAME should be a string naming the action-rule-selector.
;; "
;;     ; Check arguments
;;     (if (not (string? name))
;;         (error "Expected second argument to be a string, got: " name))
;;
;;     ; TODO: Add checks to ensure the exec-term argument is actually executable
;;     (let* ((z-name (string-append
;;                         psi-prefix-str "action-selector-" name))
;;            (selector-dsn (cog-node 'DefinedSchemaNode z-name)))
;;        (if (null? selector-dsn)
;;            (begin
;;                (set! selector-dsn (DefinedSchemaNode z-name))
;;                (DefineLink selector-dsn exec-term)
;;
;;                 (EvaluationLink
;;                     (PredicateNode "action-selector-for")
;;                     (ListLink selector-dsn (ConceptNode psi-prefix-str)))
;;
;;                 selector-dsn
;;            )
;;
;;            selector-dsn
;;        )
;;     )
;; )
;;
;; --------------------------------------------------------------
;; (define-public (psi-default-action-selector)
;; "
;;   psi-default-action-selector
;;
;;   Return highest--weighted psi-rule that is also satisfiable.
;;   If a satisfiable rule doesn't exist then the empty list is returned.
;; "
;;     (define (rule-weight RULE)
;;         (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
;;             (rule-sc-weight RULE)
;;             (rule-sa-weight RULE)
;;         )
;;     )
;;
;;    (pick-from-weighted-list
;;        (psi-get-all-satisfiable-rules)
;;        rule-weight)
;; )
;;
; ----------------------------------------------------------------------
(define-public (psi-set-action-selector exec-term demand-node)
"
  psi-set-action-selector EXEC-TERM DEMAND-NODE - Sets EXEC-TERM as
  the function used to select rules for the DEMAND-NODE.

  EXEC-TERM should be an executable atom.
  DEMAND-NODE should be any demand that has been defined.
"
    (psi-set-functionality exec-term #f demand-node "action-selector")
)

; ----------------------------------------------------------------------
(define-public (psi-get-action-selector demand-node)
"
  psi-get-action-selector DEMAND-NODE - Gets the action-selector of
  DEMAND-NODE.
"
    (psi-get-functionality demand-node "action-selector")
)

; --------------------------------------------------------------

; Sort RULE-LIST, according to the weights assigned by the WEIGHT-FN.
(define (sort-by-weight RULE-LIST WEIGHT-FN)

    (sort! RULE-LIST (lambda (RA RB)
        (> (WEIGHT-FN RA) (WEIGHT-FN RB))))
)

; --------------------------------------------------------------
; Compute the weight of RULE, as product of strength times confidence.
(define (rule-sc-weight RULE)
    (let ((rule-stv (cog-tv RULE))
          (context-stv (psi-rule-satisfiability RULE)))
        (* (tv-conf rule-stv) (tv-mean rule-stv)
           (tv-conf context-stv) (tv-conf context-stv))))

; Compute the weight of RULE, as product of strength times
; confidence times attention-value.
(define (rule-sa-weight RULE)
    (let ((a-stv (cog-tv RULE))
          (sti (assoc-ref (cog-av->alist (cog-av RULE)) 'sti)))
        (* (tv-conf a-stv) (tv-mean a-stv) sti)))

; --------------------------------------------------------------
(define (pick-from-weighted-list ALIST WEIGHT-FUNC)
"
  pick-from-weighted-list ALIST WEIGHT-FUNC

  Return a list containing an item picked from ALIST, based
  on the WEIGHT-FUNC. If ALIST is empty, then return the empty
  list. If ALIST is not empty, then randomly select from the list,
  with the most heavily weighted item being the most likely to be
  picked.  The random distribution is the uniform weighted interval
  distribution.
"
    ; Create a list of rules sorted by weight.
    (define sorted-rules (sort-by-weight ALIST WEIGHT-FUNC))

    ; Function to sum up the total weights in the list.
    (define (accum-weight rule-list)
        (if (null? rule-list) 0.0
            (+ (WEIGHT-FUNC (car rule-list))
                 (accum-weight (cdr rule-list)))))

    ; The total weight of the rule-list.
    (define total-weight (accum-weight sorted-rules))

    ; Pick a number from 0.0 to total-weight.
    (define cutoff (* total-weight
        (random:uniform (random-state-from-platform))))

    ; Recursively move through the list of rules, until the
    ; sum of the weights of the rules exceeds the cutoff.
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
        ((null? sorted-rules) '())

        ; If there's only one rule in the list, return it.
        ((null? (cdr sorted-rules)) sorted-rules)

        ; Else randomly pick among the most-weighted rules.
        (else (list (pick-rule cutoff sorted-rules))))
)

(define (default-per-demand-action-selector demand)
"
  default-per-demand-action-selector DEMAND

  Return a list containing zero or more psi-rules that can satisfy
  the DEMAND.  Zero rules are returned only if there are no rules
  that can satisfy the demand.  Usually, only one rule is returned;
  it is choosen randomly from the set of rules that could satsify
  the demand. The choice function is weighted, so that the most
  heavily-weighted rule is the one most likely to be chosen.
"
    (pick-from-weighted-list
        (psi-get-weighted-satisfiable-rules demand)
        rule-sc-weight)
)

; --------------------------------------------------------------
(define-public (psi-select-rules-per-demand d)
"
  psi-select-rules-per-demand DEMAND

  Run the action selector associated with DEMAND, and return a list
  of psi-rules.  If no custom action selector was specified, then
  a list containing a single rule will be returned; that rule will
  be highest-weight rule that is also satsisfiable.  If there is no
  such rule, then the empty list is returned.
"
    (let ((as (psi-get-action-selector d)))
        (if (null? as)
            ; If the user didn't specify a custom selector, then
            ; run the default selector
            (default-per-demand-action-selector d)

            ; Else run the user's selector.
            ; Assume the results are wrapped in
            ; a ListLink under a SetLink. Can be
            ; expanded in the future if needed
            (let ((result (gar (cog-execute! (car as)))))
                (if (null? result)
                    '()
                    (cog-outgoing-set result)
                )
            )
        )
    )


    ;(let ((demands (psi-get-all-demands)))
    ;    ;NOTE:
    ;    ; 1. If there is any hierarcy/graph, get the information from the
    ;    ;    atomspace and do it here.
    ;    ; 2. Any changes between steps are accounted for, i.e, there is no
    ;    ;    caching of demands. This has a performance penality.
    ;    ; FIXME:
    ;    ; 1. Right now the demands are not separated between those that
    ;    ;    are used for emotiong modeling vs those that are used for system
    ;    ;    such as chat, behavior, ...
    ;    (append-map select-rules demands)
    ;)
)
