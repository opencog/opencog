;; Load PLN
(use-modules (opencog pln))
(pln-load)

(use-modules (opencog logger))
(use-modules (opencog ure))
;; (cog-logger-set-level! "fine")
(ure-logger-set-level! "debug")

;; Load KB
(load "kb.scm")

;; What does Marry like?
(define what (Variable "$what"))
(define vardecl (TypedVariable what (Type "ConceptNode")))
(define sources (Set
                  song-3-composed-by-author-2
                  marry-like-song-3
                  listener-like-song-from-same-author))
(define target (Evaluation like (List marry what)))

;; Remove irrelevant rules to speed up reasoning
(pln-rm-rules-by-names (list
                        ;; Remove deduction rules
                        "deduction-implication-rule"
                        "deduction-subset-rule"
                        "deduction-inheritance-rule"
                        ;; Remove modus ponens rules
                        "modus-ponens-inheritance-rule"
                        "modus-ponens-implication-rule"
                        "modus-ponens-subset-rule"
                        ;; Remove contraposition rules
                        "crisp-contraposition-implication-scope-rule"
                        "contraposition-implication-rule"
                        "contraposition-inheritance-rule"
                        ;; Remove conjunction introduction rules but the binary one
                        "fuzzy-conjunction-introduction-1ary-rule"
                        "fuzzy-conjunction-introduction-3ary-rule"
                        "fuzzy-conjunction-introduction-4ary-rule"
                        "fuzzy-conjunction-introduction-5ary-rule"
                        ;; Remove disjunction introduction rules
                        "fuzzy-disjunction-introduction-1ary-rule"
                        "fuzzy-disjunction-introduction-2ary-rule"
                        "fuzzy-disjunction-introduction-3ary-rule"
                        "fuzzy-disjunction-introduction-4ary-rule"
                        "fuzzy-disjunction-introduction-5ary-rule"
                        ;; Remove conditional instantiation but implication scope one
                        "conditional-full-instantiation-implication-meta-rule"
                        "conditional-full-instantiation-inheritance-meta-rule"))

;; Call forward chainer
;;
;; fc-results includes, among others,
;;
;; (EvaluationLink (stv 0.9 0.5625)
;;   (PredicateNode "like")
;;   (ListLink
;;     (ConceptNode "Marry")
;;     (ConceptNode "Dextrose is my bitch")))
;;
;; Because Marry likes a song from the same author
(define fc-results
  (pln-fc sources #:maximum-iterations 20 #:fc-retry-exhausted-sources #t))

;; Call backward chainer
;;
;; The following does not work due to the lack of full support of
;; meta-rule in backward chainer.
;; 
;; (pln-bc target #:vardecl vardecl #:maximum-iterations 1000)
