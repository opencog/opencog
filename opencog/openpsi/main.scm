; Copyright (C) 2015-2016 OpenCog Foundation

(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (srfi srfi-1)) ; For `append-map`

(use-modules (opencog) (opencog exec) (opencog query) (opencog rule-engine))

(load "action-selector.scm")
(load "demand.scm")
(load "utilities.scm")

(define psi-action (Concept "OpenPsi: action"))

; --------------------------------------------------------------

(define-public (psi-rule-nocheck context action goal a-stv demand)
"
  psi-rule-nocheck -- same as psi-rule, but no checking
"
    (define implication (Implication a-stv (And context action) goal))

    ; These memberships are needed for making filtering and searching simpler..
    ; If GlobNode had worked with GetLink at the time of coding this,
    ; that might have been; better, (or not as it might need as much chasing)
    (MemberLink action psi-action)

    ; AndLink's are unordered links; must use an ordered link, if the
    ; context is to preceed the action! SequentialAnd seems like an
    ; OK choice, for now.
    (MemberLink implication demand)

    implication
)

; --------------------------------------------------------------
(define-public (psi-rule context action goal a-stv demand)
"
  psi-rule CONTEXT ACTION GOAL TV DEMAND - create a psi-rule.

  Associate an action with a context such that, if the action is
  taken, then the goal will be satisfied.  The structure of a rule
  is in the form of an `ImplicationLink`,

    (ImplicationLink TV
        (AndLink
            CONTEXT
            ACTION)
        GOAL)

  where:
  CONTEXT is a scheme list containing all of the terms that should
    be met for the ACTION to be taken. These are atoms that, when
    evaluated, should result in a true or false TV.

  ACTION is an evaluatable atom, i.e. returns a TV when evaluated by
    `cog-evaluate!`.  It should return a true or false TV.

  GOAL is an evaluatable atom, i.e. returns a TV when evaluated by
    `cog-evaluate!`.  The returned TV is used as a formula to rank
    how this rule affects the demands.

  TV is the TruthValue assigned to the ImplicationLink. It should
    be a SimpleTruthValue.

  DEMAND is a Node, representing the demand that this rule affects.
"
    (define func-name "psi-rule") ; For use in error reporting

    ; Check arguments
    (if (not (list? context))
        (error (string-append "In procedure " func-name ", expected first "
            "argument to be a list, got:") context))
    (if (not (cog-atom? action))
        (error (string-append "In procedure " func-name ", expected second "
            "argument to be an atom, got:") action))
    (if (not (cog-atom? goal))
        (error (string-append "In procedure " func-name ", expected third "
            "argument to be an atom, got:") goal))
    (if (not (cog-tv? a-stv))
        (error (string-append "In procedure " func-name ", expected fourth "
            "argument to be a stv, got:") a-stv))
    (if (not (equal? (stv 1 1) (psi-demand? demand)))
        (error (string-append "In procedure " func-name ", expected fifth "
            "argument to be a node representing a demand, got:") demand))

    (psi-rule-nocheck context action goal a-stv demand)
)

; --------------------------------------------------------------
(define-public (psi-get-rules demand-node)
"
  Returns a list of all psi-rules that affect the given demand.

  demand-node:
  - The node that represents the demand.
"
    (cog-chase-link 'MemberLink 'ImplicationLink demand-node)
)

; --------------------------------------------------------------
(define-public (psi-get-all-rules)
"
  Returns a list of all known openpsi rules.

XXX FIXME -- this is painfully slow --- multiple minutes when
there are 100K rules!
"
    (fold append '()
        (par-map (lambda (x) (cog-chase-link 'MemberLink 'ImplicationLink x))
            (psi-get-all-demands)))
)

; --------------------------------------------------------------
(define-public (psi-rule? atom)
"
  Returns `#t` or `#f` depending on whether the passed argument
  is a valid psi-rule or not. An ImplicationLink that is a member
  of the demand sets is a psi-rule.

  XXX FIXME -- this is very very slow (many minutes) when there
  are 100K or more rules!

  atom:
  - An atom passed for checking.
"
    (if (member atom (psi-get-all-rules)) #t #f)
)

; --------------------------------------------------------------
(define-public (psi-get-all-actions)
"
  Returns a list of all openpsi actions.
"
    (cog-chase-link 'MemberLink 'ListLink psi-action))

; --------------------------------------------------------------
(define-public (psi-action? ATOM)
"
  Check if ATOM is an action and return `#t`, if it is, and `#f`
  otherwise. An atom is an action if it a member of the set
  represented by (ConceptNode \"OpenPsi: action\").
"
    (let ((candidates (cog-chase-link 'MemberLink 'ConceptNode ATOM)))

        ; A filter is used to account for empty list as well as
        ; cog-chase-link returning multiple results, just in case.
        (not (null?
            (filter (lambda (x) (equal? x psi-action)) candidates)))
    )
)

; --------------------------------------------------------------
(define (get-c&a impl-link)
"
  Get context and action list from ImplicationLink.
"
    (cog-outgoing-set (list-ref (cog-outgoing-set impl-link) 0))
)

; --------------------------------------------------------------
(define-public (psi-get-context rule)
"
  Get the context of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    (remove psi-action? (get-c&a rule))
)

; --------------------------------------------------------------
(define-public (psi-get-action rule)
"
  Get the action of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    (car (filter psi-action? (get-c&a rule)))
)

; --------------------------------------------------------------
(define-public (psi-get-goal rule)
"
  Get the goal of an openpsi-rule.

  rule:
  - An openpsi-rule.
"
    ; NOTE: Why this function? -> For consisentency and to accomodate future
    ; changes
     (cadr (cog-outgoing-set rule))
)

; --------------------------------------------------------------
(define-public (psi-related-goals action)
"
  Return a list of all the goals that are associated by an action. Associated
  goals are those that are the implicand of the psi-rules that have the given
  action in the implicant.

  action:
  - An action that is part of a psi-rule.
"
    (let* ((and-links (cog-filter 'SequentialAndLink (cog-incoming-set action)))
           (rules (filter psi-rule? (append-map cog-incoming-set and-links))))
           (delete-duplicates (map psi-get-goal rules))
    )

)

; --------------------------------------------------------------
(define-public (psi-satisfiable? rule)
"
  Check if the rule is satisfiable and return TRUE_TV or FALSE_TV.

  rule:
  - A psi-rule to be checked if its context is satisfiable.
"
    ; NOTE: stv are choosen as the return values so as to make the function
    ; usable in evaluatable-terms.
    (let* ((pattern (SatisfactionLink (AndLink (psi-get-context rule))))
           (result (cog-evaluate! pattern)))
          (cog-delete pattern)
          result
    )
)

; --------------------------------------------------------------
(define-public (psi-get-satisfiable-rules demand-node)
"
  Returns a list of psi-rules of the given demand that are satisfiable.

  demand-node:
  - The node that represents the demand.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define-public (psi-get-all-satisfiable-rules)
"
  Returns a list of all the psi-rules that are satisfiable.
"
    (filter  (lambda (x) (equal? (stv 1 1) (psi-satisfiable? x)))
        (psi-get-all-rules))
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
            (most-weighted-atoms (psi-get-all-rules))
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
(define-public (psi-select-rules)
"
  Returns a list of psi-rules that are satisfiable by using the action-selector
  you defined or the default-action-selector predefined if you haven't defined
  a different action-selector.
"
    (let ((dsn (psi-get-action-selector-generic)))
        (if (null? dsn)
            (psi-default-action-selector (random-state-from-platform))
            (let ((result (cog-execute! (car dsn))))
                (if (equal? (cog-type result) 'SetLink)
                    (cog-outgoing-set result)
                    (list result)
                )
            )
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
        (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
            (most-weighted-atoms (psi-get-satisfiable-rules demand))
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
(define-public (psi-select-rules-per-demand d)
"
  Returns a list of psi-rules that are satisfiable by using the action-selector
  you defined or the default-action-selector predefined if you haven't defined
  a different action-selector.
"
    (if (equal? d (ConceptNode "OpenPsi: AIML chat demand"))
        (list) ; Skip the aiml chat demand . FIXME: this is a hack  
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

; --------------------------------------------------------------
; Main loop control
; --------------------------------------------------------------
(define psi-do-run-loop #t)

(define-public (psi-running?)
"
  Return #t if the openpsi loop is running, else return #f.
"
    psi-do-run-loop
)

; --------------------------------------------------------------
(define psi-loop-count 0)

(define-public (psi-get-loop-count)
"
 behavior-tree-loop-count

 Return the loop-count of the behavior tree.
"
    psi-loop-count
)

; --------------------------------------------------------------
(define-public (psi-run-continue?)  ; public only because its in a GPN
    (set! psi-loop-count (+ psi-loop-count 1))

    ; Pause for 101 millisecs, to keep the number of loops
    ; within a reasonable range.
    (usleep 101000)
    (if psi-do-run-loop (stv 1 1) (stv 0 1))
)

; ----------------------------------------------------------------------
(define-public (psi-set-action-executor exec-term demand-node)
"
  psi-set-action-executor EXEC-TERM DEMAND-NODE - Sets EXEC-TERM as the
  the function to be used as action-executor for the rules of DEMAND-NODE.
"
    (psi-set-functionality exec-term #f demand-node "action-executor")
)

; ----------------------------------------------------------------------
(define-public (psi-get-action-executor demand-node)
"
  psi-get-action-executor DEMAND-NODE - Gets the action-executor of
  DEMAND-NODE.
"
    (psi-get-functionality demand-node "action-executor")
)

; ----------------------------------------------------------------------
(define-public (psi-set-goal-evaluator eval-term demand-node)
"
  psi-set-goal-evaluator EVAL-TERM DEMAND-NODE - Sets EVAL-TERM as the
  the function to be used as goal-evaluator for the rules of DEMAND-NODE.
"
    (psi-set-functionality eval-term #t demand-node "goal-evaluator")
)

; ----------------------------------------------------------------------
(define-public (psi-get-goal-evaluator demand-node)
"
  psi-get-goal-evaluator DEMAND-NODE - Gets the goal-evaluator for
  DEMAND-NODE.
"
    (psi-get-functionality demand-node "goal-evaluator")
)

; ----------------------------------------------------------------------
(define-public (psi-step)
"
  The main function that defines the steps to be taken in every cycle.
"
    (define (get-context-grounding-atoms rule)
        #!
        (let* ((pattern (GetLink (AndLink (psi-get-context rule))))
                ;FIXME: Cache `results` during `psi-select-rules` stage
               (results (cog-execute! pattern)))
            (cog-delete pattern)
            ; If it is only links then nothing to pass to an action.
            (if (null? (cog-get-all-nodes results))
                '()
                results
            )))!#
            '())


    (define (act-and-evaluate rule)
        ;NOTE: This is the job of the action-orchestrator.
        (let* ((action (psi-get-action rule))
               (goals (psi-related-goals action))
               (context-atoms (get-context-grounding-atoms rule)))

            (if (null? context-atoms)
                (cog-execute! action)
                (cog-execute! (PutLink action context-atoms))
            )
            (map cog-evaluate! goals)
        ))

    (map
        (lambda (d)
            ; The assumption is that the rules can be run concurrently.
            ; FIXME: Once action-orchestrator is available then a modified
            ; `psi-select-rules` should be used insted of
            ; `psi-select-rules-per-demand`
            (map act-and-evaluate (psi-select-rules-per-demand d)))
        (psi-get-all-demands)
    )

    (stv 1 1) ; For continuing psi-run loop.
)

; --------------------------------------------------------------
(define-public (psi-run)
"
  Run `psi-step` in a new thread. Call (psi-halt) to exit the loop.
"
    (define loop-name (string-append psi-prefix-str "loop"))
    (define loop-node (DefinedPredicateNode loop-name))
    (define (define-psi-loop)
        (DefineLink
            loop-node
            (SatisfactionLink
                (SequentialAnd
                    (Evaluation
                        (GroundedPredicate "scm: psi-step")
                        (ListLink))
                    (Evaluation
                        (GroundedPredicate "scm: psi-run-continue?")
                        (ListLink))
                    ; tail-recursive call
                    loop-node))))

    (if (null? (cog-node 'DefinedPredicateNode loop-name))
        (define-psi-loop))

    (set! psi-do-run-loop #t)
    (call-with-new-thread
        (lambda () (cog-evaluate! loop-node)))
)

; -------------------------------------------------------------
(define-public (psi-halt)
"
  Tells the psi loop thread, that is started by running `(psi-run)`, to exit.
"
    (set! psi-do-run-loop #f)
)
