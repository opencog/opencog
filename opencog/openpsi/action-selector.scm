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
;;   EXE can be either an ExecutionOutputLink or a DefinedSchemaNode that
;;   represents an executable link. It will be used fto select the psi-rules
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

; --------------------------------------------------------------
(define-public (psi-add-action-selector exec-term name)
"
  Returns the DefinedSchemaNode that represents the executable term
  after defining it as an openpsi action-selector.

  exec-term:
  - An executable term.

  name:
  -  A string for naming the action-rule-selector. The name will be prefixed
     by the following string `OpenPsi: action-rule-selector-`.
"
    ; Check arguments
    (if (not (string? name))
        (error "Expected second argument to be a string, got: " name))

    ; TODO: Add checks to ensure the exec-term argument is actually executable
    (let* ((z-name (string-append
                        psi-prefix-str "action-selector-" name))
           (selector-dsn (cog-node 'DefinedSchemaNode z-name)))
       (if (null? selector-dsn)
           (begin
               (set! selector-dsn (DefinedSchemaNode z-name))
               (DefineLink selector-dsn exec-term)

                (EvaluationLink
                    (PredicateNode "action-selector-for")
                    (ListLink selector-dsn (ConceptNode psi-prefix-str)))

                selector-dsn
           )

           selector-dsn
       )
    )
)

; ----------------------------------------------------------------------
(define-public (psi-set-action-selector exec-term demand-node)
"
  psi-set-action-selector EXEC-TERM DEMAND-NODE - Sets EXEC-TERM as the
  the function to be used as action-selector for the rules of DEMAND-NODE.
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
(define-public (psi-context-weight rule)
"
  Returns the TruthValue of an evaluated context. The strength is known as
  the weight(Sc) of the context.

  rule:
  - A psi-rule with context to be evaluated.
"
    (define (context-stv stv-list)
    ; See and-side-effect-free-formula in pln-and-construction-rule
        (stv
            (fold * 1 (map (lambda (x) (tv-mean x)) stv-list))
            (fold min 1 (map (lambda (x) (tv-conf x)) stv-list)))
    )

    (let* ((context (psi-get-context rule))
        ; map-in-order is used to simulate SequentialAndLink assuming
        ; psi-get-context maintaines, which is unlikely. What other options
        ; are there?
           (stvs (map-in-order cog-evaluate! context)))

           (context-stv stvs)
   )
)

; --------------------------------------------------------------
(define-public (psi-action-weight rule)
"
  Retruns the weight of an action in a single psi-rule(Wcagi)

  rule:
  - The psi-rule that has the action, for which the action weight is being
  calcualated.
"
    ; NOTE: This check is required as ecan isn't being used continuesely.
    ; Remove `most-weighted-atoms` version once ecan is integrated.
    (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
        ; Wcagi = Scga * Sc * 1 (assuming every rule is important)
        (* (tv-mean (cog-tv rule)) ;Scga
           (tv-mean (psi-context-weight rule))) ; Sc
        ; Wcagi = Scga * Sc * STIcga
        (* (tv-mean (cog-tv rule)) ;Scga
           (tv-mean (psi-context-weight rule)) ; Sc
           (assoc-ref (cog-av->alist (cog-av rule)) 'sti)) ; STIcga
    )
)

; --------------------------------------------------------------
(define-public (psi-most-weighted-rules rule-list)
"
  It returns a list with non-duplicating rules with the highest weight. If an
  empty list is passed an empty list is returned. Weight of an psi-rule is as
  defined in `psi-action-weight` function

  rule-list:
  - A list of psi-rules to be compared.
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
