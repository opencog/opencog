;; Rule to specialize a pattern by composing it with a shallow
;; abstraction, which can be pattern with just one link and all
;; variables as outgoings, a constant, or a variable, and checks that
;; it has enough support.
;;
;; Given g with arity n and with support ms, and f, specialize g by
;; composing it with f over one of its variables, xi.
;;
;; Evaluation <tv1>
;;   Predicate "minsup"
;;   List
;;     Lambda
;;       VariableList
;;         <x1>
;;         ...
;;         <xn>
;;       <g-body>
;;     <texts>
;;     <ms>
;; Evaluation <tv2>
;;   Predicate "shallow-abstraction"
;;   List
;;     List
;;       <x1>
;;       ...
;;       <xi-1>
;;       <f>
;;       <xi+1>
;;       ...
;;       <xn>
;;     <minsup evaluation above>
;; |-
;; Evaluation <tv3>
;;   Predicate "minsup"
;;   List
;;     Put
;;       Lambda
;;         VariableList
;;           <x1>
;;           ...
;;           <xn>
;;         <g-body>
;;       List
;;         <x1>
;;         ...
;;         <xi-1>
;;         <f>
;;         <xi+1>
;;         ...
;;         <xn>
;;     <texts>
;;     <ms>
;;
;; assuming that tv1 and tv2 are equal to (stv 1 1), then calculate
;; the frequency of the composed pattern and set tv3 accordingly, (stv
;; 1 1) if g composed with f has support ms, (stv 0 1) otherwise.
;;
;; <f> may either a pattern with one link all variables as outgoings,
;; a constant or a variable amongst <x1> to <xn> different than <xi>.
;;
;; We don't need to care about the structure of (List <x1> ...)
;; because, as presented implemented, only correct structures will be
;; formed by the shallow-abstraction rule.
;;
;; TODO: we might want to split such rule into 2,
;;
;; 1. Relating PutLink and specialization/abstraction
;;
;; 2. Relating specialization/abstraction and minsup
;;
;; instead of being mangled into one rule.

(load "miner-rule-utils.scm")

(define specialization-rule
  (let* (;; Variables
         (g (Variable "$g"))
         (texts (Variable "$texts"))
         (ms (Variable "$ms"))
         (xs-f (Variable "$xs-f"))
         ;; Types
         (NumberT (Type "NumberNode"))
         (LambdaT (Type "LambdaLink"))
         (PutT (Type "PutLink"))
         (ConceptT (Type "ConceptNode"))
         ;; Vardecls
         (g-decl (TypedVariable g (TypeChoice LambdaT PutT)))
         (texts-decl (TypedVariable texts ConceptT))
         (ms-decl (TypedVariable ms NumberT))
         (xs-f-decl xs-f)
         (vardecl (VariableList g-decl texts-decl ms-decl xs-f-decl))
         ;; Clauses
         (minsup-g (minsup-eval g texts ms))
         (shabs-eval (shallow-abstraction-eval xs-f minsup-g))
         ;; Make sure the pattern has the minimum support
         (precond-1 (absolutely-true-eval minsup-g))
         (precond-2 (absolutely-true-eval shabs-eval))
         ;; Rewrite
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: specialization-formula")
                    (List
                      (minsup-eval
                        (Quote (Put
                          (Unquote g)
                          (Unquote xs-f)))
                        texts
                        ms)
                      minsup-g
                      shabs-eval))))
    (Bind
      vardecl
      (And shabs-eval precond-1 precond-2)
      rewrite)))

(define (specialization-formula conclusion . premises)
  ;; (cog-logger-debug "specialization-formula conclusion = ~a, premises = ~a"
  ;;                   conclusion premises)
  (if (= (length premises) 2)
      (let* ((con-minsup-args (gdr conclusion))
             (pre-minsup-pred (car premises))
             (pre-minsup-pred-tv (cog-tv pre-minsup-pred))
             (gf (cog-outgoing-atom con-minsup-args 0))
             (texts (cog-outgoing-atom con-minsup-args 1))
             (ms-atom (cog-outgoing-atom con-minsup-args 2))
             (ms (inexact->exact (atom->number ms-atom)))
             (conclusion-tv (if (and (tv->bool pre-minsup-pred-tv)
                                     ;; The lazyness of and allows to
                                     ;; avoid testing g.f support if g
                                     ;; doesn't have enough support
                                     ;; TODO: do you really need this?
                                     (enough-support? gf texts ms))
                                ;; Both g and g,f have enough support
                                (stv 1 1)
                                ;; f.g doesn't have enough support
                                #f))
             ;; Reduce the pattern to a normal form by collapsing all
             ;; PutLinks, and output this norma form
             (reduced-conclusion (cog-execute! conclusion)))
        (if conclusion-tv
            (cog-set-tv! reduced-conclusion conclusion-tv)))))

;; Define specialization
(define specialization-rule-name
  (DefinedSchemaNode "specialization-rule"))
(DefineLink specialization-rule-name
  specialization-rule)
