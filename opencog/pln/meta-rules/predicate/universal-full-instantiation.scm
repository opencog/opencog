;; =======================================================================
;; Universal Full Instantiation Meta Rule
;;
;; Two forms are implemented
;;
;; 1. Scoped links, like ForAllLink, LambdaLink, etc.
;;
;; <universal-quantifer-scope-link>
;;   V
;;   P
;; |-
;;   T
;;   |-
;;   P[V->T]
;;
;; 2. Unscoped nodes or links, like ConceptNode, PredicateNode,
;;    SatisfyingSetLink, etc.
;;
;; <universal-quantifer-atom>
;; |-
;;   T
;;   |-
;;   <eval-type>
;;     <universal-quantifer-atom>
;;     T
;;
;; Note that depending on <eval-type> the arguments should be
;; swapped. For instance MemberLink and EvaluationLink have swapped
;; arguments.
;;
;; Also depending on the quantifier the formula may be different. For
;; now we simply (and wrongly) paste the TV from the quantifier to the
;; produced instance.
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Conditional full instantiation rules for scope links ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO turn that into a generator

(define universal-full-instantiation-forall-1ary-meta-rule
  (let* ((X (Variable "$X"))
         (XType (Variable "$XType"))
         (VariableT (Type "VariableNode"))
         (TypeChoiceT (Type "TypeChoice"))
         (TypeT (Type "TypeNode"))
         (XTypeVardeclT (TypeChoice TypeChoiceT TypeT))
         (P (Variable "$P"))

         ;; Meta rule variable declaration
         (meta-vardecl (VariableList
                         (TypedVariable X VariableT)
                         (TypedVariable XType XTypeVardeclT)
                         P))
         ;; Meta rule main clause
         (forall (Quote
                   (ForAll
                     (Unquote
                       (TypedVariable
                         X
                         XType))
                     (Unquote P))))
         ;; Meta rule precondition
         (meta-precondition (Evaluation
                              (GroundedPredicate "scm: gt-zero-confidence")
                              forall))
         ;; Meta rule pattern
         (meta-pattern (And forall meta-precondition))

         ;; Produced rule variable declaration. P will now be content
         ;; rather than variables.
         (produced-vardecl (TypedVariable X XType))
         ;; Produced rule pattern. Look for groundings of P that meet
         ;; the precondition.
         (produced-pattern X)
         ;; Produced rule rewrite. Apply formula to calculate the TV
         ;; over P[V->T].
         (produced-rewrite (ExecutionOutput
                            (GroundedSchema "scm: universal-full-instantiation-forall-formula")
                            (Unquote
                              (List
                                ;; Conclusion
                                P
                                ;; Premise
                                forall))))
         ;; Meta rule rewrite
         (meta-rewrite (Quote (Bind
                          (Unquote produced-vardecl)
                          (Unquote produced-pattern)
                          produced-rewrite ; the Unquote appears
                                           ; inside it, to avoid
                                           ; running the
                                           ; ExecutionOutput
                          ))))
    (Bind
      meta-vardecl
      meta-pattern
      meta-rewrite)))

(define (universal-full-instantiation-forall-formula Pinst Forall)
  (let* ((Forall-tv (cog-tv Forall)))
    (if (< 0 (cog-tv-confidence Forall-tv)) ; avoid creating informationless knowledge
        (cog-merge-hi-conf-tv! Pinst Forall-tv))))

;; Name the forall scope meta rule
(define universal-full-instantiation-forall-1ary-meta-rule-name
  (DefinedSchemaNode "universal-full-instantiation-forall-1ary-meta-rule"))
(DefineLink universal-full-instantiation-forall-1ary-meta-rule-name
  universal-full-instantiation-forall-1ary-meta-rule)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Universal full instantiation rules for non-scope atoms ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; TODO
