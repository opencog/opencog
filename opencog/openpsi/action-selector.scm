;
; action-selector.scm
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
;;             (psi-default-action-selector (random-state-from-platform))
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
(define (psi-most-weighted-rule rule-list)
"
  psi-most-weighted-rule RULE-LIST

  Return the single rule from hte list having the highest weight.
  The weight of an psi-rule is as defined in `psi-action-weight` function
"
    (define (pick rule lst) ; prev is a `lst` and next `atom`
        (cond
            ((> (psi-action-weight (car lst)) (psi-action-weight rule)) lst)
            ((= (psi-action-weight (car lst)) (psi-action-weight rule))
                (append lst (list rule)))
            (else (list rule))))

    (if (null? rule-list)
        '()
        (delete-duplicates (fold pick (list (car rule-list)) rule-list))
    )
)

; --------------------------------------------------------------
(define-public (psi-default-action-selector a-random-state)
"
  Returns a list of one of the most-important-weighted and satisfiable psi-rule
  or an empty list. A single psi-rule is returned so as help avoid mulitple
  actions of the same effect or type(aka semantic of the action) from being
  executed. If a satisfiable rule doesn't exist then the empty list is returned.

  a-random-state:
  - A random-state object used as a seed for choosing how multiple satisfiable
  psi-rules with the same weight are to be choosen.
"
    (define (choose-rules)
        ; NOTE: This check is required as ecan isn't being used continuesely.
        ; Remove `most-weighted-atoms` version once ecan is integrated.
        (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
            (most-weighted-atoms (psi-get-all-satisfiable-rules))
            (most-important-weighted-atoms (psi-get-all-satisfiable-rules))
        )
    )

    (let ((rules (choose-rules)))
        (if (null? rules)
            '()
            (list (list-ref rules (random (length rules) a-random-state)))
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-default-action-selector-per-demand a-random-state demand)
"
  Returns a list of one of the most-important-weighted and satisfiable psi-rule
  or an empty list. A single psi-rule is returned so as help avoid mulitple
  actions of the same effect or type(aka semantic of the action) from being
  executed. If a satisfiable rule doesn't exist then the empty list is returned.

  a-random-state:
  - A random-state object used as a seed for choosing how multiple satisfiable
  psi-rules with the same weight are to be choosen.
"
    (define (choose-rules)
        ; NOTE: This check is required as ecan isn't being used continuesely.
        ; Remove `most-weighted-atoms` version once ecan is integrated.
        ; FIXME; Replace by
        ; (psi-most-weighted-rules (psi-get-satisfiable-rules demand))
        ;(if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
            (most-weighted-atoms (psi-get-weighted-satisfiable-rules demand))
            ;(most-important-weighted-atoms (psi-get-all-satisfiable-rules))
        ;)
    )

    (let ((rules (choose-rules)))
        (cond
            ((null? rules) '())
            ((equal? (tv-mean (cog-tv (car rules))) 0.0) '())
            (else
                (list (list-ref rules (random (length rules) a-random-state))))
        )
    )
)

; --------------------------------------------------------------
(define-public (psi-select-rules-per-demand d)
"
  psi-select-rules-per-demand DEMAND

  Returnxs a list of psi-rules that can be satisfied by the current
  action-selector.
"
    (let ((as (psi-get-action-selector d)))
        (if (null? as)
            (psi-default-action-selector-per-demand
                       (random-state-from-platform) d)
            (let ((result (cog-execute! (car as))))
                (if (equal? (cog-type result) 'SetLink)
                    (cog-outgoing-set result)
                    (list result)
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
