(use-modules (opencog nlp oc)) ; Needed for TenseLink

; Helper for generating a new BindLink with the base elements
(define (generate-bindlink . elmts)
  (define vars (append (car sq-base) (append-map car elmts)))
  (define pats (append (cadr sq-base) (append-map cadr elmts)))
  (define rws (append (caddr sq-base) (append-map caddr elmts)))
  (Bind (VariableList vars) (And pats) (Set rws)))

(define sq-base
  (list
    (list
      (TypedVariable (Variable "$sent") (Type "SentenceNode"))
      (TypedVariable (Variable "$parse") (Type "ParseNode"))
      (TypedVariable (Variable "$tense") (Type "DefinedLinguisticConceptNode"))
      (TypedVariable (Variable "$word-inst-p") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$pred-inst") (Type "PredicateNode"))
      (TypedVariable (Variable "$pred") (Type "PredicateNode")))
    (list
      (State (Anchor "GHOST Currently Processing") (Variable "$sent"))
      (Parse (Variable "$parse") (Variable "$sent"))
      (WordInstance (Variable "$word-inst-p") (Variable "$parse"))
      (Reference (Variable "$pred-inst") (Variable "$word-inst-p"))
      (Implication (Variable "$pred-inst") (Variable "$pred"))
      (TenseLink (Variable "$word-inst-p") (Variable "$tense")))
    (list
      (Inheritance
        (InterpretationNode "MicroplanningNewSentence")
        (DefinedLinguisticConcept "InterrogativeSpeechAct"))
      (Inheritance
        (Variable "$pred")
        (ExecutionOutput
          (GroundedSchema "scm: tense-to-inf")
          (List (Variable "$tense")))))))

(define sq-subj
  (list
    (list
      (TypedVariable (Variable "$word-inst-s") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$subj-inst") (Type "ConceptNode")))
    (list
      (WordInstance (Variable "$word-inst-s") (Variable "$parse"))
      (Reference (Variable "$subj-inst") (Variable "$word-inst-s"))
      (Evaluation
        (DefinedLinguisticRelationship "_subj")
        (List (Variable "$word-inst-p") (Variable "$word-inst-s"))))
    (list
      (Evaluation
        (Variable "$pred")
        (List (Variable "$subj-inst"))))))

(define sq-obj
  (list
    (list
      (TypedVariable (Variable "$word-inst-o") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$obj-inst") (Type "ConceptNode")))
    (list
      (WordInstance (Variable "$word-inst-o") (Variable "$parse"))
      (Reference (Variable "$obj-inst") (Variable "$word-inst-o"))
      (Evaluation
        (DefinedLinguisticRelationship "_obj")
        (List (Variable "$word-inst-p") (Variable "$word-inst-o"))))
    (list
      (Evaluation
        (Variable "$pred")
        (List (Variable "$subj-inst") (Variable "$obj-inst"))))))

(define sq-det
  (list
    (list
      (TypedVariable (Variable "$word-inst-d") (Type "WordInstanceNode")))
    (list
      (Evaluation
        (DefinedLinguisticRelationship "_det")
        (List (Variable "$word-inst-o") (Variable "$word-inst-d"))))
    (list
      (Evaluation
        (DefinedLinguisticPredicate "definite")
        (List (Variable "$obj-inst"))))))

(define sq-nn
  (list
    (list
      (TypedVariable (Variable "$word-inst-nn") (Type "WordInstanceNode"))
      (TypedVariable (Variable "$nn-inst") (Type "ConceptNode")))
    (list
      (Reference (Variable "$nn-inst") (Variable "$word-inst-nn"))
      (Evaluation
        (DefinedLinguisticRelationship "_nn")
        (List (Variable "$word-inst-o") (Variable "$word-inst-nn"))))
    (list
      (Inheritance (Variable "$obj-inst") (Variable "$nn-inst")))))

; e.g. she runs
(define bind-subj (generate-bindlink sq-subj))

; e.g. he builds houses
(define bind-subj-obj (generate-bindlink sq-subj sq-obj))

; e.g. it likes those apples
(define bind-subj-obj-det (generate-bindlink sq-subj sq-obj sq-det))

; e.g. I eat chocolate cakes
(define bind-subj-obj-nn (generate-bindlink sq-subj sq-obj sq-nn))

; e.g. we found that stone cave
(define bind-subj-obj-det-nn (generate-bindlink sq-subj sq-obj sq-det sq-nn))

(define-public (tense-to-inf dlcn)
"
  tense-to-inf DLCN

  DLCN is a DefinedLinguisticConceptNode, denoting the tense of a predicate.

  This function turns present/past tense into present_infinitive/past_infinitive
  tense. It's just for the stochastic question generator.
"
  (define tense (cog-name dlcn))

  ; To turn a statement, e.g. "she runs", into a question, e.g.
  ; "why does she run", requires turning the tense from present/past
  ; to present-infinitive/past-infinitive, and this should be done
  ; before passing it to SuReal.
  (cond
    ((string=? "present" tense) (DefinedLinguisticConcept "present_infinitive"))
    ((string=? "past" tense) (DefinedLinguisticConcept "past_infinitive"))
    (else dlcn))
)
