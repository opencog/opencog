;
; rule.scm
; Utilities for defining and working with OpenPsi rules.
;
; Copyright (C) 2016 OpenCog Foundation
; Copyright (C) 2017 MindCloud
;
; Design Notes:
; Aliases: Rules can be given a "name", called an "alias", below.
; Certain parts of the design assume that a rule has only one name,
; but other parts of the code pass around lists of names. This can
; lead to crazy bugs, due to a lack of checking or enforcement of a
; one-name-per-rule policy.  Also -- each rule should, in principle
; have a unique name; however, the current chatbot uses the same name
; for all chat rules.


(use-modules (ice-9 threads)) ; For `par-map`
(use-modules (ice-9 optargs)) ; For `define*`
(use-modules (srfi srfi-1)) ; For `drop-right`, `append-map`, etc.
(use-modules (opencog) (opencog query))

(load "demand.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define psi-action (ConceptNode (string-append psi-prefix-str "action")))
(define psi-rule-name-predicate-node
    (PredicateNode (string-append psi-prefix-str "rule_name")))

(define psi-goal-node (ConceptNode (string-append psi-prefix-str "goal")))

; --------------------------------------------------------------
(define (psi-goal NAME)
"
  psi-goal NAME

  Create and return a ConceptNode that represents an OpenPsi goal.
  The NAME should be a string.
"
  ; NOTE: Why not make this part of psi-rule function? Because, developers
  ; might want to specify the behavior they prefer, when it comes to how
  ; to measure the level of achivement of goal, and how the goal's measurement
  ; value should change.
  (let* ((goal-node (ConceptNode NAME)))
    (InheritanceLink goal-node psi-goal-node)
    goal-node
  )
)

; --------------------------------------------------------------
(define (psi-goal? ATOM)
"
  Check if ATOM is a goal and return `#t`, if it is, and `#f`
  otherwise. An atom is a goal if it a member of the set
  represented by (ConceptNode \"OpenPsi: goal\").
"
    (not (null?  (cog-link 'InheritanceLink ATOM psi-goal-node)))
)

; --------------------------------------------------------------
(define (psi-get-rules demand-node)
"
  Returns a list of all psi-rules that affect the given demand.

  demand-node:
  - The node that represents the demand.
"
    (cog-chase-link 'MemberLink 'ImplicationLink demand-node)
)

; --------------------------------------------------------------
(define (psi-get-all-rules)
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
(define (psi-rule? atom)
"
  psi-rule? ATOM

  Returns `#t` or `#f`, depending on whether the passed argument
  is a valid psi-rule or not. A psi-rule is any ImplicationLink that
  is a member of a demand set.

  ATOM is the atom to check.
"
    (let ((candidates (cog-chase-link 'MemberLink 'ConceptNode atom)))

        ; A filter is used to account for empty list as well as
        ; cog-chase-link returning multiple results, just in case.
        (not (null?
            (filter psi-demand? candidates)))
    )
)

; --------------------------------------------------------------
(define (psi-action? ATOM)
"
  Check if ATOM is an action and return `#t`, if it is, and `#f`
  otherwise. An atom is an action if it a member of the set
  represented by (ConceptNode \"OpenPsi: action\").
"
    (not (null?  (cog-link 'MemberLink ATOM psi-action)))
)

; --------------------------------------------------------------
(define (psi-rule-set-alias psi-rule name)
"
  psi-rule-set-alias PSI-RULE NAME

  NAME is a string that will be associated with the rule and is used
    as a repeatable means of referring to the rule. The hash of the
    rule can't be used, because it is a function of the Type of atoms,
    and there is no guarantee that the relationship between Types is
    static as a result of which the hash-value could change.
"
; TODO: Change to value
    (EvaluationLink
        psi-rule-name-predicate-node
        (ListLink
            psi-rule
            (ConceptNode (string-append psi-prefix-str name))))
)
; --------------------------------------------------------------
(define (psi-rule-alias psi-rule)
"
  psi-rule-alias RULE

  Returns a list of the aliases associated with the psi-rule RULE.
  An alias is just a name that is associated with the rule; it
  just provides an easy way to refer to a rule.
"
    ;; Using a GetLink here is quite inefficient; this would run much
    ;; faster if cog-chase-link was used instead. FIXME.
    (cog-outgoing-set (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "rule-alias")
                (TypeNode "ConceptNode"))
            (EvaluationLink
                psi-rule-name-predicate-node
                (ListLink
                    (QuoteLink psi-rule)  ;; ?? why is a Quote needed here?
                    (VariableNode "rule-alias"))))))
)

; --------------------------------------------------------------
(define (psi-partition-rule-with-alias rule-alias psi-rule-list)
"
  psi-partition-rule-with-alias ALIAS RULE-LIST

  Partitions the RULE-LIST into two lists: the first list will
  contain all rules that are tagged with the name ALIAS; the second
  list are all the other rules.  ALIAS should be a string.
"
    (define cali (Concept (string-append psi-prefix-str rule-alias)))
    (partition
        (lambda (rule) (equal? cali (car (psi-rule-alias rule))))
        psi-rule-list)
)

; --------------------------------------------------------------
(define (psi-related-goals action)
"
  psi-related-goals ACTION

  Return a list of all of the goals that are associated with the ACTION.
  A goal is associated with an action whenever the goal appears in
  a psi-rule with that action in it.
"
    ;; XXX this is not an efficient way of searching for goals.  If
    ;; this method is used a lot, we should search for the goals directly.
    (let* ((and-links (cog-incoming-by-type action 'SequentialAndLink))
           (rules (filter psi-rule?  (append-map
                    (lambda (sand) (cog-incoming-by-type sand 'ImplicationLink))
                  and-links))))
           (delete-duplicates (map psi-get-goal rules))
    )
)

; --------------------------------------------------------------
; Utility wrapper for above; returns crisp #t or #f
(define (is-satisfiable? RULE)
    (equal? (stv 1 1) (psi-satisfiable? RULE))
)

(define (psi-rule-satisfiability rule)
"
  psi-rule-satisfiability RULE

  Given the RULE, return the probability that the RULE can be satisfied.
  XXX Except this doesn't return a probability, it just returns TRUE_TV
  or FALSE_TV.
  XXX fixme - this replaces psi-satisfiable?
"
; NOTE
; 1. See https://github.com/opencog/atomspace/issues/823 for why
;   psi-satisfiable? is used
; 2. Should a context evaluator be added here?????
; 3. What is the "right" way of communicating the level of information.
    (satisfiable? rule)
)

; --------------------------------------------------------------
(define (psi-get-satisfiable-rules demand-node)
"
  psi-get-satisfiable-rules DEMAND

  Returns a list of all of the psi-rules associated with the DEMAND
  that are also satisfiable.
"
    (filter is-satisfiable? (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define (psi-get-weighted-satisfiable-rules demand-node)
"
  psi-get-weighted-satisfiable-rules DEMAND

  Returns a list of all the psi-rules associated with DEMAND
  that are satisfiable and have a non-zero strength.
"
    (filter
        (lambda (x)
            (and (> (cog-stv-strength x) 0)
                (is-satisfiable? x)))
        (psi-get-rules demand-node))
)

; --------------------------------------------------------------
(define (psi-get-all-satisfiable-rules)
"
  psi-get-all-satisfiable-rules

  Returns a list of all the psi-rules that are satisfiable.
"
    (filter is-satisfiable? (psi-get-all-rules))
)

; --------------------------------------------------------------
(define (psi-get-all-weighted-satisfiable-rules)
"
  psi-get-all-weighted-satisfiable-rules

  Returns a list of all the psi-rules that are satisfiable and
  have a non-zero strength.
"
    (filter
        (lambda (x) (and (> (cog-stv-strength x) 0)
            (is-satisfiable? x)))
        (psi-get-all-rules))
)

; --------------------------------------------------------------
(define (psi-context-weight rule)
"
  psi-context-weight RULE

  Return a TruthValue containing the weight Sc of the context of the
  psi-rule RULE.  The strength of the TV will contain the weight.
"
    (define (context-stv stv-list)
        ; See and-side-effect-free-formula in pln-and-construction-rule
        ; XXX FIXME explain what is being computed here.
        (stv
            (fold * 1 (map (lambda (x) (tv-mean x)) stv-list))
            (fold min 1 (map (lambda (x) (tv-conf x)) stv-list)))
    )

    ; map-in-order is used to simulate SequentialAndLink assuming
    ; psi-get-context maintains, which is unlikely. What other options
    ; are there?
    ; XXX FIXME How about actually using a SequentialAndLink?
    ; then teh code will be faster, and tehre won't be this problem.
    (context-stv (map-in-order cog-evaluate! (psi-get-context rule)))
)

;; --------------------------------------------------------------
;; DEAD CODE not used anywhere
;;
;; (define (psi-action-weight rule)
;; "
;;   psi-action-weight RULE
;;
;;   Return the weight Wcagi of the action of the psi-rule RULE.
;; "
;;     ; NOTE: This check is required as ecan isn't being used continuously.
;;     ; Remove `most-weighted-atoms` version once ecan is integrated.
;;     ; XXX FIXME this is just-plain wrong. ECAN and OpenPsi have nothing
;;     ; to do with each other. Nothing in OpenPsi should depend on ECAN.
;;     (if (or (equal? 0 (cog-af-boundary)) (equal? 1 (cog-af-boundary)))
;;         ; Wcagi = Scga * Sc * 1 (assuming every rule is important)
;;         (* (tv-mean (cog-tv rule)) ;Scga
;;            (tv-mean (psi-context-weight rule))) ; Sc
;;         ; Wcagi = Scga * Sc * STIcga
;;         (* (tv-mean (cog-tv rule)) ;Scga
;;            (tv-mean (psi-context-weight rule)) ; Sc
;;            (assoc-ref (cog-av->alist (cog-av rule)) 'sti)) ; STIcga
;;     )
;; )
;;
;; --------------------------------------------------------------
;; DEAD CODE not used anywhere
;;
;; (define (psi-most-weighted-rule rule-list)
;; "
;;   psi-most-weighted-rule RULE-LIST
;;
;;   Return the single rule from hte list having the highest weight.
;;   The weight of an psi-rule is as defined in `psi-action-weight` function
;; "
;;     (define (pick rule lst) ; prev is a `lst` and next `atom`
;;         (cond
;;             ((> (psi-action-weight (car lst)) (psi-action-weight rule)) lst)
;;             ((= (psi-action-weight (car lst)) (psi-action-weight rule)) lst)
;;             (else (list rule))))
;;
;;     (if (null? rule-list)
;;         '()
;;         (delete-duplicates (fold pick (list (car rule-list)) rule-list))
;;     )
;; )
