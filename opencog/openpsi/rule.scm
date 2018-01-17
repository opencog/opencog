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
; Used as a key for naming rules
(define psi-rule-name-predicate-node (Predicate "alias"))
; Used to declare the set of actions.
(define psi-action-node (Concept "action"))
; Used to declare the set of goals.
(define psi-goal-node (Concept "goal"))

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
  represented by (ConceptNode \"goal\").
"
  (not (null?  (cog-link 'InheritanceLink ATOM psi-goal-node)))
)

; --------------------------------------------------------------
(define (psi-get-rules category)
"
  psi-get-rules CATEGORY
    Returns a list of all psi-rules that are member of CATEGORY.
"
  (cog-chase-link 'MemberLink 'ImplicationLink category)
)

; --------------------------------------------------------------
(define (psi-action? ATOM)
"
  psi-action? ATOM
    Check if ATOM is an action and return `#t`, if it is, and `#f`
    otherwise. An atom is an action if it a member of the set
    represented by (ConceptNode \"action\").
"
  (not (null?  (cog-link 'MemberLink ATOM psi-action)))
)

; --------------------------------------------------------------
(define (psi-rule-set-alias! rule name)
"
  psi-rule-set-alias! RULE NAME
    NAME is a string that will be associated with the rule and is used
    as a repeatable means of referring to the rule. The hash of the
    rule can't be used, because it is a function of the Type of atoms,
    and there is no guarantee that the relationship between Types is
    static; as a result of which the hash-value could change.
"
  (EvaluationLink
    psi-rule-name-predicate-node
    (ListLink
      rule
      (ConceptNode (string-append psi-prefix-str name))))

  ; TODO Uncomment after testing with ghost
  ;(cog-set-value!
  ;  rule
  ;  psi-rule-name-predicate-node
  ;  (StringValue name))
)
; --------------------------------------------------------------
(define (psi-rule-alias rule)
"
  psi-rule-alias RULE
    Returns the alias of the RULE, if it was set, or null if not.
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
          (QuoteLink rule)  ;; ?? why is a Quote needed here?
          (VariableNode "rule-alias"))))))

  ; TODO Uncomment after testing with ghost
  ;(let ((alias-value (cog-value rule psi-rule-name-predicate-node)))
  ;  (if (null? alias-value)
  ;    '()
  ;    (cog-value-ref alias-value 0)
  ;  )
  ;)
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
  ;; this method is used a lot, we should search for the goals directly
  ;; See https://github.com/opencog/opencog/pull/2899.
  (let* (
    (and-links (cog-incoming-by-type action 'AndLink))
    (rules (filter psi-rule?  (append-map
      (lambda (sand) (cog-incoming-by-type sand 'ImplicationLink))
          and-links))))

    (delete-duplicates (map psi-get-goal rules))
  )
)

; --------------------------------------------------------------
(define (is-satisfiable? rule)
"
  is-satisfiable? RULE
    Returns crisp #t or #f depending on whether RULE is satisfiable
    or not.
"
  (equal? (stv 1 1) (psi-satisfiable? rule))
)

; --------------------------------------------------------------
(define (psi-get-satisfiable-rules category)
"
  psi-get-satisfiable-rules CATEGORY
    Returns a SetLink of all of the psi-rules that are member of CATEGORY
    and are satisfiable.
"
  (Set (filter is-satisfiable? (psi-get-rules category)))
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
    ; Createas an stv which is the comonentwise product of all the stvs in
    ; stv-list, for the purpose of estimating the truthvalue of a
    ; context when it is true.
    (stv
      (fold * 1 (map (lambda (x) (tv-mean x)) stv-list))
      (fold min 1 (map (lambda (x) (tv-conf x)) stv-list)))
  )

  ; map-in-order is used to simulate AndLink assuming
  ; psi-get-context maintains, which is unlikely. What other options
  ; are there?
  ; XXX FIXME How about actually using a SequentialAndLink?
  ; then the code will be faster, and there won't be this problem.
  ; TODO: This calculation can be done in OpenPsiImplicator::grounding or
  ; when the rule is being added. Since it unlikely to change except
  ; during learning it can be saved in the AndLink.
  (context-stv (map-in-order cog-evaluate! (psi-get-context rule)))
)
